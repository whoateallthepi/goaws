package main

import (
	"fmt"
	"machine"

	"math"
	"time"

	"github.com/whoateallthepi/tinygodrivers/bme280spi"
	"github.com/whoateallthepi/tinygodrivers/ds3231"
	"github.com/whoateallthepi/tinygodrivers/ledpanel"
	"github.com/whoateallthepi/tinygodrivers/rak8nn"
)

// Program control constants
const (
	debounce                = time.Millisecond * 100 //
	windMultiplier  float32 = 2.4                    // one windclick per second = 2.4km/h
	rainMultiplier  float32 = 0.2794                 // one click of the rain gauge = .2794 mm of rain
	alarmNumber             = 2
	reportFrequency         = 10 // Every 10 minutes - should be a divisor of 60
)

// hardware pins
const (
	windSpeedPin machine.Pin = machine.GP2
	rainPin      machine.Pin = machine.GP4
	windADC                  = machine.ADC0 // GP26
	batteryADC               = machine.ADC1 // GP27
	led0         machine.Pin = machine.GP13 // TX - red
	led1         machine.Pin = machine.GP14 // Status - green
	led2         machine.Pin = machine.GP15 // RX yellow
	sensorSPI    machine.Pin = machine.GP17
	i2cSDA       machine.Pin = machine.GP20
	i2cSCL       machine.Pin = machine.GP21
)

// mathmatical constants
const (
	pi = 3.14159265
)

// Interface for loraWan
type lorawan rak8nn.Networker

// Interface for sensor
type sensor bme280spi.Sensor

// Interface for clock
type clock ds3231.Clocker // Interface

// All the values needed for a weather report
type weatherReportDetails struct {
	temperature      float32
	humidity         float32
	pressure         float32
	mslp             float32
	currentWind      windVector
	dailyGust        windVector
	twoMinuteAverage windVector
	tenMinuteGust    windVector
	rainToday        float32
	rain1hour        float32
	rainSinceLast    float32
	battery          float32
}

//====================== Messages ==============================================

type messageHeaderOut struct {
	timeStamp [8]byte // these are all numbers of hex chars
	timeZone  [2]byte
}

type weatherReport struct {
	header         messageHeaderOut
	windDir        [3]byte
	windSpeed      [4]byte
	windGust       [4]byte
	windGustDir    [3]byte
	windSpeed2m    [4]byte
	windDir2m      [3]byte
	windGust10m    [4]byte
	windGustDir10m [3]byte
	humidity       [4]byte
	temperature    [4]byte
	rain1h         [4]byte
	rainToday      [4]byte
	rainSinceLast  [4]byte
	barUncorrected [4]byte
	barCorrected   [4]byte
	battery        [4]byte
}

type stationReport struct {
	header    messageHeaderOut
	latitude  [8]byte
	longitude [8]byte
	altitude  [4]byte
}

// status is used for error reporting - see statusErr
type status int

// statusErr is an implementation of error interface to include a status integer
type statusErr struct {
	status  status
	message string
}

// Error codes
const (
	noSensor = iota + 1
	sensorReadError
	noWindADC
	failedSend
	clockError
	untrappedError
)

type windVector struct {
	speed float32
	angle float32
}

type emptyStruct struct{}

func (se statusErr) Error() string {
	return se.message
}

const (
	signalBoot ledpanel.Control = ledpanel.VeryLong | ledpanel.OneFlash | 0b00000111
)

func main() {

	for i := 0; i < 10; i++ {
		fmt.Printf("hello\n")
		time.Sleep(time.Second)
	}

	// Various counters....

	var windSpeeds [120]windVector // two-minute record of windspeeds
	var windGust10m [10]windVector // used to get fastest gust in past 10
	// minutes
	var dailyWindGust windVector // maximum gust today

	var rainHour [60]float32  // rainfall for each of the past 60 minutes
	var rainSinceLast float32 // rain since last report
	var rainToday float32     //reset at midnight
	// for debouncing
	var lastRainInterrupt, lastWindInterrupt time.Time

	// ======================= Initialisation ==================================
	// pins
	/*const windSpeedPin machine.Pin = machine.GP2

	//windSpeedPin := machine.GP2
	rainPin := machine.GP4
	//windDirection := machine.GP26
	windADC := machine.ADC0
	//battery := machine.GP27
	batteryADC := machine.ADC1
	//alarmPin := machine.GP3
	*/
	// leds
	// core led = led1 (green), rx led = led2 (yellow), tx led = led0(red)
	panel := ledpanel.Panel{Pins: ledpanel.Pins{led0,
		led1,
		led2},
		Durations: ledpanel.FlashDurations{},
	}
	leds, _ := ledpanel.Configure(panel) // returns a channel
	// flash all three leds to confirm boot
	f := signalBoot // Three leds, one long flash
	leds <- f

	//configure - interrupt pins
	windSpeedPin.Configure(machine.PinConfig{Mode: machine.PinInputPullup})
	rainPin.Configure(machine.PinConfig{Mode: machine.PinInputPullup})

	//configure - I2C for clock
	machine.I2C0.Configure(machine.I2CConfig{SCL: i2cSCL,
		SDA: i2cSDA,
	})

	//configure SPI for bme280
	machine.SPI0.Configure(machine.SPIConfig{})

	// create a channels for windclicks, rainclicks
	windClick := make(chan emptyStruct, 1) // buffer just in case
	rainClick := make(chan emptyStruct, 1) // do not want interrrups blocking
	sync := make(chan time.Time)           // used for resyncing to the ds3231 clock
	//alarmSound := make(chan emptyStruct, 1)
	//report := make(chan emptyStruct)

	done := make(chan emptyStruct)

	// i2c for clock device
	cd := ds3231.New(machine.I2C0) // device implementing clocker interface
	clock.Configure(&cd)

	// SPI for sensor
	sens := bme280spi.NewSPI(sensorSPI, machine.SPI0, 1)
	err := sensor.Configure(sens)
	if err != nil {
		fmt.Printf("Error configuring sensor: %s\n", err)
	}

	// GPIO interrupt routines
	windSpeedPin.SetInterrupt(machine.PinFalling, func(p machine.Pin) {
		now := time.Now()
		if now.Sub(lastWindInterrupt) > debounce {
			// Record
			windClick <- emptyStruct{}
			//fmt.Println("Windspeed")
			lastWindInterrupt = now
		}

	})
	rainPin.SetInterrupt(machine.PinFalling, func(p machine.Pin) {
		now := time.Now()
		if now.Sub(lastRainInterrupt) > debounce {
			// Record
			rainClick <- emptyStruct{}
			//fmt.Println("Rain")
			lastRainInterrupt = now
		}
	})

	// ADCs
	machine.InitADC()
	windSensor := machine.ADC{Pin: windADC}

	batterySensor := machine.ADC{Pin: batteryADC}

	rtc, err := clock.Read(&cd)
	if err != nil {
		fmt.Printf("failed to read clock: %s", err)
	}
	//======================== main processing ================================
	secondTicker := time.NewTicker(time.Second)
	var windClicks uint8

	// Indexes to various arrays
	var seconds2m, seconds, minutes, minutes10m uint8

	realTime := rtc

	for {
		select {
		case <-secondTicker.C: /// this is the every second code
			// Keep track of real time
			realTime = realTime.Add(time.Second)
			windDir, err := getWindDirection(windSensor)
			if err != nil {
				fmt.Printf("error from wind sensor: %s\n", err)
			}
			windSp := windMultiplier * float32(windClicks)
			windSpeeds[seconds2m].angle = windDir
			windSpeeds[seconds2m].speed = windSp
			seconds2m++
			if seconds2m > 119 {
				seconds2m = 0
			}

			fmt.Printf("windClicks: %d\n", windClicks)
			fmt.Printf("Wind speed: %3.1f\n", windSp)
			fmt.Printf("Wind dir: %3.1f\n", windDir)

			// Is this the fastest gust in the current minute??
			if windSp > windGust10m[minutes10m].speed {
				windGust10m[minutes10m].speed = windSp
				windGust10m[minutes10m].angle = windDir
			}

			// Fastest gust today?
			if windSp > dailyWindGust.speed {
				dailyWindGust.speed = windSp
				dailyWindGust.angle = windDir
			}
			seconds++
			if seconds > 59 {
				seconds = 0
				minutes++
				if minutes > 59 {
					minutes = 0
				}
				minutes10m++
				if minutes10m > 9 {
					minutes10m = 0
				}
				// A new minute - need to reset rainclick for current minute
				// and max gust for current minute
				rainHour[minutes] = 0
				windGust10m[minutes10m].speed = 0
				windGust10m[minutes10m].angle = 0
			}

			windClicks = 0 // reset for next second

			// diag
			fmt.Printf("Wind angle: %2.3f radians. Wind Speed: %2.3f kph\n",
				windDir, windSp)
			fmt.Printf("Rain today: %2.2f \n", rainToday)

			// Check if minutes divides exactly against reporting frequency
			if realTime.Second() == 0 {
				rem := realTime.Minute() % reportFrequency
				if rem == 0 { // time for a weather report
					go reportWeather(sens,
						batterySensor,
						windSp,
						windDir,
						dailyWindGust,
						&windSpeeds,  // Ideally we'd pass as copies
						&windGust10m, // but causes a hardware fault (stack?)
						rainToday,
						&rainHour,
						&rainSinceLast, //reset by reportweather
					)
				}

			}

		case <-windClick:
			windClicks++
		case <-rainClick:
			rainToday += rainMultiplier
			rainSinceLast += rainMultiplier
			rainHour[minutes] += rainMultiplier
		case tt := <-sync:
			realTime = tt
		case <-done:
			return
		}

	}

}

func reportWeather(d *bme280spi.DeviceSPI, // temp sensor
	b machine.ADC, // battery level
	ws float32,
	wd float32,
	dailyGust windVector,
	windSpeeds *[120]windVector,
	gust10 *[10]windVector,
	rainToday float32,
	rainHour *[60]float32,
	rainSinceLast *float32) error {

	fmt.Printf("report weather *********\n")
	var r weatherReportDetails
	sr, err := sensor.Read(d)
	if err != nil {
		return statusErr{
			status:  sensorReadError,
			message: fmt.Sprintf("failed to read sensor: %s", err),
		}
	}

	r.temperature = float32(sr.Temperature) / 1000
	r.pressure = float32(sr.Pressure) / 100000
	r.humidity = float32(sr.Humidity) / 100
	r.currentWind.speed = ws
	r.currentWind.angle = wd
	r.dailyGust = dailyGust

	// Two minute average wind
	twoMin := windSpeeds[:] // make a slice
	r.twoMinuteAverage, _ = windVectorAverage(twoMin)

	// 10 min max gust
	for _, v := range gust10 {
		if v.speed > r.tenMinuteGust.speed {
			r.tenMinuteGust.speed = v.speed
			r.tenMinuteGust.angle = v.angle
		}
	}

	// rainfall
	r.rainToday = rainToday
	// 1h total
	for _, v := range rainHour {
		r.rain1hour += v
	}
	// Since last
	r.rainSinceLast = *rainSinceLast
	*rainSinceLast = 0 // reset - maybe do this after a successful report??

	r.battery, _ = getBatteryLevel(b)

	fmt.Printf("Weather report\n %+v\n", r)

	return nil
}

func getWindDirection(windsensor machine.ADC) (float32, error) {
	const (
		sectors float32 = 16
	)
	var sector float32

	adc := windsensor.Get()

	switch {
	case adc < 0x7D0:
		return 0, statusErr{
			status:  noWindADC,
			message: fmt.Sprintf("wind sensor error - value: %d\n", adc)}
	case adc < 0x1344:
		sector = 5
	case adc < 0x16C7:
		sector = 3
	case adc < 0x1B75:
		sector = 4
	case adc < 0x2598:
		sector = 7
	case adc < 0x3276:
		sector = 6
	case adc < 0x3CFD:
		sector = 9
	case adc < 0x4CFA:
		sector = 8
	case adc < 0x5E22:
		sector = 1
	case adc < 0x70D9:
		sector = 2
	case adc < 0x813A:
		sector = 11
	case adc < 0x8B5D:
		sector = 10
	case adc < 0x9A92:
		sector = 15
	case adc < 0xA70C:
		sector = 0
	case adc < 0xB130:
		sector = 13
	case adc < 0xBCE2:
		sector = 14
	case adc < 0xCB20:
		sector = 12
	default:
		return 0, statusErr{
			status:  noWindADC,
			message: fmt.Sprintf("wind sensor error - value: %d\n", adc)}
	}
	return 2 * pi / sectors * sector, nil // 2 Pi radians = 16 sectors (full circle)
}

// getBatteryLevel takes the ADC reading and returns an estimate for the battery
// voltage.
func getBatteryLevel(battery machine.ADC) (float32, error) {
	const (
		offset  float32 = 960
		divisor float32 = 2923
	)

	b := battery.Get()
	return ((float32(b) - offset) / divisor), nil
}

// windVectorAverag takes a slice of windVectors and calculates an average speed
// (simple mean) and direction (a vector sum).
//
// This is my encoding of the method at www.noaa.gov/windav.shtml.
//
// Each direction and speed pair is broken down into
// an east-west component and a north-south component.
// Directions are in Radians clockwise from north, so the
// north-south component is the cosine of the angle times the speed.
// The east-west is the sine of the direction * speed. Then add up
// all the north-south components (NSc) and the east west (EWc)
// components (separately). The average direction is the arctan
// of  (sum EWc / sum NSc). For reasons I don't understand,
// it's better to use the arctan2 call with sum of EWc as the first
// parameter, sum of NSc as the second (something to do with
// less likely to be confused about quadrants. The second parameter
// can't be zero, so make it 360, if that happens.

func windVectorAverage(winds []windVector) (windVector, error) {

	var nSc, eWc, sumSpeeds, avgDir float64
	// nSc is sum of north-south components
	// eWc is the east-west components
	for _, v := range winds {
		nSc += math.Cos(float64(v.angle)) * float64(v.speed)
		eWc += math.Sin(float64(v.angle)) * float64(v.speed)
		sumSpeeds += float64(v.speed)
	}

	// atanf is meaningless if both components are zero
	fmt.Printf("...sum nSc = %.2f\n", nSc)
	fmt.Printf("...sum eWc = %.2f\n", eWc)

	if (eWc == 0) && (nSc == 0) {
		avgDir = 0
	} else {
		avgDir = math.Atan2(eWc, nSc)
	}

	if avgDir < 0 {
		avgDir += (2 * pi) // Atan returns between - Pi
		// and + Pi ( -180 to + 180 degrees)
	}

	return windVector{speed: float32(sumSpeeds / float64(len(winds))),
		angle: float32(avgDir)}, nil

}

func radiansToDegrees(radians float32) uint16 {

	return uint16(math.Round(float64(radians * (180 / (pi)))))

}
