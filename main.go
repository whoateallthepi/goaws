package main

import (
	"fmt"
	"machine"
	"time"

	"github.com/whoateallthepi/tinygodrivers/rak8nn"
	//"tinygo.org/x/drivers/bme280"
	"github.com/whoateallthepi/tinygodrivers/ledpanel"
)

// Program control constants
const (
	debounce               = time.Millisecond * 100 //
	windMultiplier float32 = 2.4                    // one windclick per second = 2.4km/h
	rainMultiplier float32 = 0.2794                 // one click of the rain gauge = .2794 mm of rain
)

// Interface for loraWan
type lorawan rak8nn.Networker

// status is used for error reporting - see StatusErr
type status int

// statusErr is an implementation of error interface to include a Status integer
type statusErr struct {
	status  status
	message string
}

// Error codes
const (
	noSensor = iota + 1
	noWindADC
	failedSend
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
	// Various counters....

	var windSpeeds [120]windVector // two-minute record of windspeeds
	var windGust10m [10]windVector //  used to get fastest gust in past 10 minutes
	var dailyWindGust windVector   // maximum gust today

	var rainHour [60]float32  // rainfall for each of the past 60 minutes
	var rainSinceLast float32 // rain since last report
	var rainToday float32     //reset at midnight

	// pins
	windSpeedPin := machine.GP2
	rainPin := machine.GP4
	//windDirection := machine.GP26
	windADC := machine.ADC0
	//battery := machine.GP27
	batteryADC := machine.ADC1

	// leds
	// core led = led1 (green), rx led = led2 (yellow), tx led = led0(red)
	panel := ledpanel.Panel{Pins: ledpanel.Pins{machine.GP13, machine.GP14, machine.GP15},
		Durations: ledpanel.FlashDurations{},
	}
	leds, _ := ledpanel.Configure(panel) // returns a channel
	// flash all three leds to confirm boot
	f := signalBoot // Three leds, one long flash
	leds <- f

	// for debouncing
	var lastRainInterrupt, lastWindInterrupt time.Time

	//configure
	windSpeedPin.Configure(machine.PinConfig{Mode: machine.PinInputPullup})
	rainPin.Configure(machine.PinConfig{Mode: machine.PinInputPullup})

	// create a channels for windclicks, rainclicks
	windClick := make(chan emptyStruct, 1) // buffer just in case
	rainClick := make(chan emptyStruct, 1) // do not want interrrups blocking
	done := make(chan emptyStruct)

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

	// kick off the routine for seconds processing
	secondTicker := time.NewTicker(time.Second)

	go secondProcessing(secondTicker, windClick, rainClick, done, &windSpeeds,
		&windGust10m, &dailyWindGust, &rainHour, &rainToday, &rainSinceLast, windSensor)

	for {

		b, err := getBatteryLevel(batterySensor)
		if err != nil {
			fmt.Printf("Battery ADC error: %s\n", err)
		}

		d, err := getWindDirection(windSensor)
		if err != nil {
			fmt.Printf("Wind ADC error: %s\n", err)
		}

		fmt.Printf("Wind angle: %2.3f radians. Battery: %2.3f volts\n", d, b)
		time.Sleep(time.Second * 5)

	}
	/*
		//time.Sleep(time.Hour * 24) // long wait!

		d, err := rak8nn.NewDevice("RAK811", 0, 115200)

		if err != nil {
			fmt.Println(err)
			return
		}

		err = lorawan.Join(d)

		if err != nil {
			fmt.Println(err)
			return
		}

		var data []byte = []byte("aabbcc")
		var cha uint8 = 100

		r, err := lorawan.Send(d, data, cha)

		if err != nil {
			fmt.Println(err)
			return
		}

		fmt.Println(r)

		/*func readBME280(d bme280.Device) (temperature, pressure, humidity int32, err error) {
		  	if !d.Connected() {
		  		return 0, 0, 0, statusErr{
		  			status:  noSensor,
		  			message: "BME280 not connected",
		  		}
		  	}
		  	t, _ := d.ReadTemperature()
		  	p, _ := d.ReadPressure()
		  	h, _ := d.ReadHumidity()

		  	return t, p, h, nil
		  }
	*/
}

// secondProcessing does most of the work recording interrupts from the rain and wind speed sensors
func secondProcessing(t *time.Ticker, wind <-chan emptyStruct, rain <-chan emptyStruct, done <-chan emptyStruct,
	twoMinWind *[120]windVector, gust10m *[10]windVector, dailyGust *windVector,
	rainHour *[60]float32, rainToday *float32, rainSinceLast *float32, ws machine.ADC) error {

	var windClicks uint8

	// Indexes to various arrays
	var seconds2m, seconds, minutes, minutes10m uint8

	for {
		select {
		case <-t.C: /// this is the every second code

			windDir, err := getWindDirection(ws)
			if err != nil {
				fmt.Printf("error from wind sensor: %s\n", err)
			}
			windSp := windMultiplier * float32(windClicks)
			twoMinWind[seconds2m].angle = windDir
			twoMinWind[seconds2m].speed = windSp
			seconds2m++
			if seconds2m > 119 {
				seconds2m = 0
			}

			fmt.Printf("windClicks: %d\n", windClicks)
			fmt.Printf("Wind speed: %3.1f\n", windSp)
			fmt.Printf("Wind dir: %3.1f\n", windDir)

			// Is this the fastest gust in the current minute??
			if windSp > gust10m[minutes10m].speed {
				gust10m[minutes10m].speed = windSp
				gust10m[minutes10m].angle = windDir
			}

			// Fastest gust today?
			if windSp > dailyGust.speed {
				dailyGust.speed = windSp
				dailyGust.angle = windDir
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
				gust10m[minutes10m].speed = 0
				gust10m[minutes10m].angle = 0
			}

			windClicks = 0 // reset for next second

		case <-wind:
			windClicks++
		case <-rain:
			//

			*rainToday += rainMultiplier
			*rainSinceLast += rainMultiplier
			rainHour[minutes] += rainMultiplier

		case <-done:
			return nil
		}

	}
}

// getWindDirection takes the ADC reading and returns an angle in radians.
// The various steps are calculated from the Sparkfun Weather Meters docs - see
// https://cdn.sparkfun.com/assets/d/1/e/0/6/DS-15901-Weather_Meter.pdf
// Be careful as there have been several versions of the meters with slightly different
// resistance values.
func getWindDirection(windsensor machine.ADC) (float32, error) {
	const (
		pi      float32 = 3.1416
		sectors float32 = 16
	)
	var sector float32

	adc := windsensor.Get()

	switch {
	case adc < 0x7D0:
		return 0, statusErr{
			status:  noWindADC,
			message: fmt.Sprintf("wind sensor disconnected/ read error - returned value: %d\n", adc)}
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
			message: fmt.Sprintf("wind sensor disconnected/ read error - returned value: %d\n", adc)}
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
