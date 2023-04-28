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
	debounce               = time.Millisecond * 10
	windMultiplier float32 = 2.4    // one windclick per second = 2.4km/h
	rainMultiplier float32 = 0.2794 // one click of the gauge = .2794 mm
	// alarmNumber             = 2
	reportFrequency = 10 // Every 10 minutes - should be a divisor of 60
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

// defaults
const (
	defaultLatitude  = 0.00
	defaultLongitude = 0.00
	defaultAltitude  = 100
	defaultTimeZone  = 0
)

// baselines are used to minimize the number of bytes sent by the
// station. These are added back in to the readings by the base station
const (
	baselineTime        = 1640995200 // 2022-01-01 00:00:00 UTC
	baselinePressure    = 900        // This is in millibars
	baselineTemperature = 50.00      // added to temperature to avoid negatives
)

// network statuses
const (
	networkUnknown = iota
	deviceError
	noLoraWan
	joinedLoraWan
)

// Interface for loraWan
type lorawan rak8nn.Networker

// Interface for sensor
type sensor bme280spi.Sensor

// Interface for clock
type clock ds3231.Clocker // Interface

type windVector struct {
	speed float32
	angle float32
}

// Information about this station
type stationData struct {
	timeZone      int8
	latitude      float32
	longitude     float32
	altitude      int16
	networkStatus int8
	//sendStationReport int8
}

func (s stationData) String() string {
	return fmt.Sprintf("timezone: %d, lat: %2.5f, long: %2.5f, alt: %d",
		s.timeZone, s.latitude, s.longitude, s.altitude)
}

// All the values needed for a weather report
type weatherReport struct {
	epochTime        int64
	timeZone         int8 // currently this is full HOURS only
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

// Implement the stringer interface on weatherReport so we can use
// fmt.Sprintf to quickly format the output message

func (w weatherReport) String() string {
	str, err := formatWeatherReport(w)

	if err != nil {
		str = "error formatting weather report\n"
	}
	return str
}

// ====================== Messages =============================================
// message types
const (
	weatherData = 100
)

type messageHeaderOut struct {
	timeStamp [8]byte // these are all numbers of hex chars
	timeZone  [2]byte
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
	noNetworkDevice
	sensorReadError
	noWindADC
	failedToSend
	clockError
	untrappedError
)

type emptyStruct struct{}

func (se statusErr) Error() string {
	return se.message
}

// led codes
const (
	// three leds
	signalBoot ledpanel.Control = ledpanel.VeryLong | ledpanel.OneFlash | 0b00000111
	// red and yellow - three flashes
	loraWanOK ledpanel.Control = ledpanel.Long |
		ledpanel.ThreeFlash | ledpanel.Led0 | ledpanel.Led2
	// four flashes yellow and red
	loraWanFail ledpanel.Control = ledpanel.Long | ledpanel.FourFlash |
		ledpanel.Led0 | ledpanel.Led2
	// two flashes - green led
	sensorOK ledpanel.Control = ledpanel.Long |
		ledpanel.TwoFlash | ledpanel.Led1
	// four flashes red
	sensorFail ledpanel.Control = ledpanel.FourFlash |
		ledpanel.Medium | ledpanel.Led0 | ledpanel.Led1
	txError ledpanel.Control = ledpanel.FourFlash |
		ledpanel.Long | ledpanel.Led0
	txSuccess ledpanel.Control = ledpanel.VeryLong | ledpanel.OneFlash |
		ledpanel.Led0
		//four flashes RX (yellow)
	rxError ledpanel.Control = ledpanel.Medium | ledpanel.Led2 |
		ledpanel.FourFlash
)

func main() {
	/*
		for i := 0; i < 10; i++ {
			fmt.Printf("hello\n")
			time.Sleep(time.Second)
		}
	*/
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
		leds <- sensorFail
	}
	leds <- sensorOK

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

	// stationData
	station := stationData{timeZone: 0,
		altitude:      defaultAltitude,
		longitude:     defaultLongitude,
		latitude:      defaultLatitude,
		networkStatus: networkUnknown}

	//======================== Connect to network ==============================
	// need to think about error handling
	n, err := rak8nn.NewDevice("RAK811", 0, 115200)

	if err != nil {
		fmt.Printf("failed to connect to network device %s\n", err)
	}

	station.networkStatus = noLoraWan

	err = lorawan.Join(n)
	if err != nil {
		fmt.Printf("failed to connect to network  %s\n", err)
		leds <- loraWanFail
	} else {
		fmt.Printf("connected to network \n")
	}
	leds <- loraWanOK
	station.networkStatus = joinedLoraWan

	//======================== main processing =================================
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
			/*
				fmt.Printf("windClicks: %d\n", windClicks)
				fmt.Printf("Wind speed: %3.1f\n", windSp)
				fmt.Printf("Wind dir: %3.1f\n", windDir)
			*/
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
			/*
				// diag
				fmt.Printf("Wind angle: %2.3f radians. Wind Speed: %2.3f kph\n",
					windDir, windSp)
				fmt.Printf("Rain today: %2.2f \n", rainToday)
			*/
			// Check if minutes divides exactly against reporting frequency
			if realTime.Second() == 0 {
				rem := realTime.Minute() % reportFrequency
				if rem == 0 { // time for a weather report
					go reportWeather(sens,
						batterySensor,
						n,  // network device
						cd, // clock device
						realTime.Unix(),
						&station,
						leds,
						sync,
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
				if realTime.Minute() == 0 {
					// check for midnight processing
					if realTime.Add(
						time.Duration(station.timeZone)*time.Hour).Hour() == 00 {
						// it is midnight we need to reset but make sure
						// report weather has had time to do it's job
						go func() {
							fmt.Printf("**** midnight reset")
							time.Sleep(time.Second * 10)
							rainToday = 0
							dailyWindGust.speed = 0
							dailyWindGust.angle = 0
							time.Sleep(time.Second * 30)
							// resync the clock as the internal tick
							// drifts a few seconds a day off the RTC
							t, err := clock.Read(&cd)
							if err != nil {
								fmt.Printf("failed to read clock: %s", err)
								// no point erroring we are in a go routine
								return
							}
							sync <- t // resync to t
						}()

					}
				} // end every 00 minute checks
			} // evey minute

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
	n *rak8nn.Device, // network device
	cd ds3231.Device,
	e int64, // Unix Time
	sd *stationData,
	leds chan<- ledpanel.Control,
	sync chan<- time.Time,
	ws float32,
	wd float32,
	dailyGust windVector,
	windSpeeds *[120]windVector,
	gust10 *[10]windVector,
	rainToday float32,
	rainHour *[60]float32,
	rainSinceLast *float32) error {

	fmt.Printf("report weather *********\n")
	var r weatherReport
	sr, err := sensor.Read(d)
	if err != nil {
		return statusErr{
			status:  sensorReadError,
			message: fmt.Sprintf("failed to read sensor: %s", err),
		}
	}
	/*
		fmt.Printf("bar: %d\n Humidity: %d \n Temperature: %d\n", sr.Pressure,
			sr.Humidity, sr.Temperature)
	*/
	r.epochTime = e - baselineTime
	r.timeZone = sd.timeZone
	r.temperature = baselineTemperature + float32(sr.Temperature)/100
	r.pressure = float32(sr.Pressure)/100 - baselinePressure
	r.mslp = compensatePressure(sr.Pressure, sd.altitude, r.temperature) -
		baselinePressure
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

	db, err := lorawan.Send(n, []byte(fmt.Sprintf("%s", r)), weatherData)
	if err != nil {
		leds <- txError
		fmt.Printf("****tx error: %s\n", err)
		return statusErr{status: failedToSend,
			message: fmt.Sprintf("failed to send to lorawan: %s", err)}

	}
	leds <- txSuccess
	//fmt.Printf("datablock: %+v", db)

	if db == nil {
		fmt.Printf("****rx error: nil datablock - this should not happen")
		return nil
	}
	if db.Bytes == 0 {
		return nil
	}
	err = processDownload(db, sd, leds, cd, sync)
	if err != nil {
		return err // this will be ignored - in a goroutine
	}
	return nil
}

//=========================== Formatting messages ==============================

func formatWeatherReport(w weatherReport) (string, error) {

	o := format8Bytes(w.epochTime) +
		formatNbytes(float32(w.timeZone), 2) +
		formatWindDirection(w.currentWind.angle) +
		formatNbytes(w.currentWind.speed*100, 4) +
		formatNbytes(w.dailyGust.speed*100, 4) +
		formatWindDirection(w.dailyGust.angle) +
		formatNbytes(w.twoMinuteAverage.speed*100, 4) +
		formatWindDirection(w.twoMinuteAverage.angle) +
		formatNbytes(w.tenMinuteGust.speed*100, 4) +
		formatWindDirection(w.tenMinuteGust.angle) +
		formatNbytes(w.humidity*100, 4) +
		formatNbytes(w.temperature*100, 4) +
		formatNbytes(w.rain1hour*100, 4) +
		formatNbytes(w.rainToday*100, 4) +
		formatNbytes(w.rainSinceLast*100, 4) +
		formatNbytes(w.pressure*100, 4) +
		formatNbytes(w.mslp*100, 4) +
		formatNbytes(w.battery*100, 4)

	return o, nil

}

// ========================== Incoming messages =================================
// Message types are:
// 200 - set timezone -
// 201 - set station data
// 202 - request station report
// 203 - reboot station
func processDownload(db *rak8nn.DataBlock,
	s *stationData,
	leds chan<- ledpanel.Control, cd ds3231.Device, sync chan<- time.Time) error {

	switch db.Channel {
	case 200:
		if len(db.Data) < 4 {
			fmt.Printf("Incoming message: %d - invaild length: %d\n",
				db.Channel, len(db.Data))
			leds <- rxError
			return nil
		}

		s.timeZone = hex2int8(db.Data[0:2])

		ss := hex2int8(db.Data[2:4])
		t, err := clock.Read(&cd)
		if err != nil {
			return statusErr{
				status:  clockError,
				message: fmt.Sprintf("failed to read clock: %s", err),
			}
		}
		sd, _ := time.ParseDuration(fmt.Sprintf("%ds", ss))

		t = t.Add(sd)
		err = clock.Set(&cd, t)
		if err != nil {
			return statusErr{
				status:  clockError,
				message: fmt.Sprintf("failed to set clock: %s", err),
			}
		}
		/*
			fmt.Printf("Incoming message: %d , timezone: %d, seconds adjust: %d\n",
			  db.Channel, hex2int8(db.Data[0:2]), ss)
		*/
		// Going to resync clock, but wait a few seconds to avoid
		// too much jumping over minute boundaries
		time.Sleep(time.Second * 30)
		t, err = clock.Read(&cd)
		if err != nil {
			return statusErr{
				status:  clockError,
				message: fmt.Sprintf("failed to read clock: %s", err),
			}
		}
		sync <- t // resync to t
		return nil
	case 201:
		if len(db.Data) < 20 {
			fmt.Printf("Incoming message: %d - invaild length: %d\n",
				db.Channel, len(db.Data))
			leds <- rxError
			return nil
		}
		s.latitude = float32(hex2int32(db.Data[0:8])) / 100000 // five decimals
		s.longitude = float32(hex2int32(db.Data[8:16])) / 100000
		s.altitude = hex2int16(db.Data[16:20])

		//fmt.Printf("Incoming message: %d , lat: %s, long: %s, alt: %s\n",
		//	db.Channel, db.Data[0:8], db.Data[8:16], db.Data[16:20])
		//fmt.Printf("Station Data: %s\n", s)
	case 202:
		// no data
		fmt.Printf("Incoming message: %d\n", db.Channel)
	case 203:
		fmt.Printf("Incoming message: %d\n", db.Channel)
	default:
		// This is an error
		fmt.Printf("Incoming message: %d - unrecognised channel\n", db.Channel)
		leds <- rxError
	}
	return nil
}

//========================== Utilities and sensors =============================

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
	/*
		fmt.Printf("...sum nSc = %.2f\n", nSc)
		fmt.Printf("...sum eWc = %.2f\n", eWc)
	*/
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

func formatNbytes(f float32, n uint8) string {
	f2 := int16(math.Round(float64(f))) // two implied decimals
	formatter := fmt.Sprintf("%%0%dx", n)
	return fmt.Sprintf(formatter, f2)
}

func format4Bytes(f float32) string {
	// returns 4 bytes
	f2 := int16(math.Round(float64(f)))
	return fmt.Sprintf("%04x", f2)
}

func format8Bytes(i int64) string {
	return fmt.Sprintf("%08x", i)
}

func formatWindDirection(angle float32) string {
	d := radiansToDegrees(angle)
	return fmt.Sprintf("%03x", d)
}

func compensatePressure(pressure uint32, altitude int16,
	temperature float32) float32 {
	var pressureMb float32 = float32(pressure) / 100 // convert to millibars
	/*
		fmt.Printf("pressure: %2.2f\n altitude: %d\n temperature: %2.2f\n", pressureMb,
			altitude, temperature)

		fmt.Printf("calculated %2.2f\n", pressureMb+(pressureMb*9.80665*(float32(altitude))/
			(287*(273+temperature+float32(altitude)/400))))
	*/
	return pressureMb + (pressureMb * 9.80665 * (float32(altitude)) /
		(287 * (273 + temperature + float32(altitude)/400)))

}

// takes 2 hex digits and convert to an int8
func hex2int8(hh []byte) int8 {
	if len(hh) != 2 {
		// nonsense
		return 0
	}
	value := getNum(hh[0])
	value <<= 4
	value += getNum(hh[1])
	if value > 127 {
		value -= 128
		value -= 128
		//fmt.Printf("Value is: %d\n", value)
		return int8(value)
	}
	return int8(value)
}

// hex2int32 takes an 8 character slice of hex chars and returns a int32
// includes 2s complement for -ve nos
func hex2int32(hh []byte) int32 {
	if len(hh) != 8 {
		// nonsense
		return 0
	}
	var value uint32
	for _, v := range hh {
		value <<= 4 // doesn't matter 1st time
		value += uint32(getNum(v))
	}
	if value > 0x7FFFFFFF {
		// This should be a 2s complement -ve
		value ^= 0xFFFFFFFF
		value++
		return int32(0 - value)
	}
	return int32(value)
}

// hex2int16 takes a 4 character slice of hex chars and returns a int32
// includes 2s complement for -ve nos
// eg F234 => -3532
func hex2int16(hh []byte) int16 {
	if len(hh) != 4 {
		// nonsense
		return 0
	}
	var value uint16
	for _, v := range hh {
		value <<= 4 // doesn't matter 1st time
		value += uint16(getNum(v))
	}
	if value > 0x7FFF {
		// This should be a 2s complement -ve
		value ^= 0xFFFF
		value++
		return int16(0 - value)
	}
	return int16(value)
}

/*
int16_t hex2int16(char *hex)
{
    // takes 4 hex digits and convert to an int16

    uint16_t value = 0;

    for (int x = 0; x < 4; x++)
    {
        value <<= 4; // won't matter the firt time thru
        value += getNum(*(hex + x));
    }
    return value;
}
int32_t hex2int32(char *hex)
{
    // takes 8 hex digit and convert to an int32
    // currently no validation

    uint32_t value = 0;

    for (int x = 0; x < 8; x++)
    {
        value <<= 4; // won't matter the firt time thru
        value += getNum(*(hex + x));
    }
    return value;
}

int hex2int2sComplement(char *hex)
{
    // takes 2 hex digits and convert to an int
    int value = 0;
    value = getNum(*hex);
    value <<= 4;
    value += getNum(*(hex + 1));
    if (value > 127) return (value - 256); else return value;
}

*/
// getNum gets numeric value of a hex byte
func getNum(ch byte) byte {

	if ch >= '0' && ch <= '9' {
		return (ch - 0x30)
	}
	switch {
	case ch == 'A' || ch == 'a':
		return 10
	case ch == 'B' || ch == 'b':
		return 11
	case ch == 'C' || ch == 'c':
		return 12
	case ch == 'D' || ch == 'd':
		return 13
	case ch == 'E' || ch == 'e':
		return 14
	case ch == 'F' || ch == 'f':
		return 15
	default:
		return 0
	}
}
