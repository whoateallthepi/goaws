package main

import (
	"fmt"
	"machine"
	"runtime/volatile"

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
	flashInterval   = 20 // Flash green led (led1) every n seconds
)

// hardware pins
const (
	windSpeedPin machine.Pin = machine.GP2
	rainPin      machine.Pin = machine.GP3
	windADC                  = machine.ADC0 // GP26
	batteryADC               = machine.ADC1 // GP27
	led0         machine.Pin = machine.GP13 // TX - red
	led1         machine.Pin = machine.GP14 // Status - green
	led2         machine.Pin = machine.GP15 // RX yellow
	sensorSPI    machine.Pin = machine.GP17
	i2cSDA       machine.Pin = machine.GP20
	i2cSCL       machine.Pin = machine.GP21
	rebootPin    machine.Pin = machine.GP22 // Base of NPN, C to RUN, E to earth
)

// mathmatical constants
const (
	pi = 3.14159265
)

// defaults
const (
	defaultLatitude  = 0.00
	defaultLongitude = 0.00
	defaultAltitude  = 196
	defaultTimeZone  = 0
)

// baselines are used to minimize the number of bytes sent by the
// station. These are added back in to the readings by the base station
const (
	baselineTime        = 1640995200 // 2022-01-01 00:00:00 UTC
	baselinePressure    = 900        // This is in millibars
	baselineTemperature = 50.00      // added to temperature - avoids negatives
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

// windvector stores a speed and direction. This implementation uses
// km/h and radians.
type windVector struct {
	speed float32
	angle float32
}

// stationData includes important information about this station
// altitude should be set correctly (in metres above/below sea level) as
// this is used to calculate barometric pressure adjusted to sea level
type stationData struct {
	timeZone          int8
	latitude          float32
	longitude         float32
	altitude          int16
	networkStatus     int8
	sendStationReport bool
}

func (s stationData) String() string {
	return fmt.Sprintf("timezone: %d, lat: %2.5f, long: %2.5f, alt: %d",
		s.timeZone, s.latitude, s.longitude, s.altitude)
}

// weatherReport includes all the values needed for a weather report
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

// stationReport is a struct with all the values needed for a station report
type stationReport struct {
	epochTime int64
	timeZone  int8 // currently this is full HOURS only
	latitude  float32
	longitude float32
	altitude  int16
}

// Implement the stringer interface on stationReport so we can use
// fmt.Sprintf to quickly format the output message
func (s stationReport) String() string {
	str, err := formatStationReport(s)

	if err != nil {
		str = "error formatting station report\n"
	}
	return str
}

// message types
const (
	weatherMessage = 100
	stationMessage = 101
)

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
	loraWanOK ledpanel.Control = ledpanel.VeryLong |
		ledpanel.OneFlash | ledpanel.Led0 | ledpanel.Led2
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
	aliveFlash ledpanel.Control = ledpanel.Short | ledpanel.OneFlash |
		ledpanel.Led1
	aliveNoNetwork ledpanel.Control = ledpanel.Short |
		ledpanel.OneFlash | ledpanel.Led1 | ledpanel.Led0
)

// =============================== main ========================================
func main() {
	//diagnostics
	for i := 0; i < 10; i++ {
		fmt.Printf("hello\n")
		time.Sleep(time.Second)
	}
	//
	//=========================== deferred functions ===========================
	// Reboot pin
	rebootPin.Configure(machine.PinConfig{Mode: machine.PinOutput})
	rebootPin.Low() // setting high will earth the run pin and force a reload

	// Last hope recovery procedure. Note recover isn't fully implemented
	// in TinyGo. See
	// https://tinygo.org/docs/reference/lang-support/#a-note-on-the-recover-builtin
	//
	// Commented out - causing stack overflow??
	defer func() {
		// Safest bet is to force a hardware reload
		rebootPin.High()
	}()

	//========================== variables =====================================
	//
	var windSpeeds [120]windVector // two-minute record of windspeeds
	var windGust10m [10]windVector // used to get fastest gust in past 10
	// minutes
	var dailyWindGust windVector // maximum gust today

	var rainHour [60]float32  // rainfall for each of the past 60 minutes
	var rainSinceLast float32 // rain since last report
	var rainToday float32     //reset at midnight
	// for debouncing
	var lastRainInterrupt, lastWindInterrupt time.Time
	var windClicks uint8

	// ======================= Initialisation ==================================
	//
	panel := ledpanel.Panel{Pins: ledpanel.Pins{led0,
		led1,
		led2},
		Durations: ledpanel.FlashDurations{},
	}
	leds, _ := ledpanel.Configure(panel) // returns a channel
	// flash all three leds to confirm boot
	f := signalBoot // Three leds, one long flash
	leds <- f
	//time.Sleep(time.Second)
	//configure - interrupt pins
	windSpeedPin.Configure(machine.PinConfig{Mode: machine.PinInputPullup})
	rainPin.Configure(machine.PinConfig{Mode: machine.PinInputPullup})

	//configure - I2C for clock
	machine.I2C0.Configure(machine.I2CConfig{SCL: i2cSCL,
		SDA: i2cSDA,
	})

	//configure SPI for bme280
	machine.SPI0.Configure(machine.SPIConfig{})

	// create a channels for windclicks, rainclicks, alive
	windClick := make(chan emptyStruct, 1) // buffer just in case
	rainClick := make(chan emptyStruct, 1) // do not want interrrups blocking
	sync := make(chan emptyStruct, 1)      // used for resyncing to the ds3231 clock
	alive := make(chan emptyStruct, 1)

	done := make(chan emptyStruct)

	// i2c for clock device
	cd := ds3231.New(machine.I2C0) // device implementing clocker interface
	clock.Configure(&cd)

	// SPI for sensor
	sens := bme280spi.NewSPI(sensorSPI, machine.SPI0, 1)
	err := sensor.Configure(sens)
	if err != nil {
		fmt.Printf("Error configuring sensor\n")
		leds <- sensorFail
	} else {
		leds <- sensorOK
	}
	// GPIO interrupt routines
	windSpeedPin.SetInterrupt(machine.PinFalling, func(p machine.Pin) {
		now := time.Now()
		if now.Sub(lastWindInterrupt) > debounce {
			// Record
			w := volatile.LoadUint8(&windClicks)
			w++
			volatile.StoreUint8(&windClicks, w)
			//windClick <- emptyStruct{}
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
		altitude:          defaultAltitude,
		longitude:         defaultLongitude,
		latitude:          defaultLatitude,
		sendStationReport: true,
		networkStatus:     networkUnknown}

	//======================== Connect to network ==============================
	//
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

	//======================== main loop =======================================
	//
	secondTicker := time.NewTicker(time.Second)

	// Indexes to various arrays
	var seconds2m, seconds, minutes, minutes10m uint8

	realTime := rtc

	for { // handles the various action from channels
		select {
		// Second ticker
		case <-secondTicker.C: /// this is the every second code
			// Keep track of real time
			realTime = realTime.Add(time.Second)
			windDir, err := getWindDirection(windSensor)
			if err != nil {
				fmt.Printf("error from wind sensor: %s\n", err)
			}
			w := volatile.LoadUint8(&windClicks)
			windSp := windMultiplier * float32(w)
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
			volatile.StoreUint8(&windClicks, 0) // reset for next second
			/*
				// diag
				fmt.Printf("Wind angle: %2.3f radians. Wind Speed: %2.3f kph\n",
					windDir, windSp)
				fmt.Printf("Rain today: %2.2f \n", rainToday)
			*/
			// Check if minutes divides exactly against reporting frequency
			if realTime.Second() == 0 {
				rem := realTime.Minute() % reportFrequency
				if rem == 0 {
					go func() {
						fmt.Printf("report weather *********\n")

						var r weatherReport
						sr, err := sensor.Read(sens)
						if err != nil {
							fmt.Printf("failed to read sensor\n")
							return
						}
						/*
							fmt.Printf("bar: %d\n Humidity: %d \n Temp: %d\n",
							    sr.Pressure,
								sr.Humidity,
								sr.Temperature)
						*/
						r.epochTime = realTime.Unix() - baselineTime
						r.timeZone = station.timeZone
						r.temperature = baselineTemperature +
							float32(sr.Temperature)/100
						r.pressure = float32(sr.Pressure)/100 - baselinePressure
						r.mslp = compensatePressure(sr.Pressure,
							station.altitude, r.temperature) - baselinePressure
						r.humidity = float32(sr.Humidity) / 100
						r.currentWind.speed = windSp
						r.currentWind.angle = windDir
						r.dailyGust = dailyWindGust

						// Two minute average wind
						twoMin := windSpeeds[:] // make a slice
						r.twoMinuteAverage, _ = windVectorAverage(twoMin)

						// 10 min max gust
						for _, v := range windGust10m {
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
						r.rainSinceLast = rainSinceLast
						rainSinceLast = 0 // reset - maybe after a successful report??

						r.battery, _ = getBatteryLevel(batterySensor)

						err = sendReport(n, cd, &station, leds, sync,
							[]byte(fmt.Sprintf("%s", r)),
							weatherMessage)

						if err != nil {
							fmt.Printf("Failed to send. Data: %s, Error: %s\n",
								r,
								err)

							return // error would be ignored - in a goroutine
						}
						return
					}()
				}

				if realTime.Minute() == 0 {
					// check for midnight processing
					dst := realTime.Add(
						time.Duration(station.timeZone) * time.Hour)
					fmt.Printf("Real Time: %s,Timezone %d, Calculated DST : %s\n",
						realTime.String(), station.timeZone, dst.String())
					if realTime.Add(
						time.Duration(station.timeZone)*time.Hour).Hour() == 00 {
						// it is midnight we need to reset but make sure
						// report weather has had time to do it's job
						go func() {
							time.Sleep(time.Second * 10)
							fmt.Printf("**** midnight reset\n")
							rainToday = 0
							dailyWindGust.speed = 0
							dailyWindGust.angle = 0
							time.Sleep(time.Second * 20)
							// resync the clock as the internal tick
							// drifts a few seconds a day off the RTC
							sync <- emptyStruct{} // resync to t
							return
						}()
					}
				} // end every 00 minute checks
			} else if realTime.Second() == 30 && station.sendStationReport {
				// something has requested a station report ( message 101)
				// These go out at 30s past the minute to avoid clashes
				go func() {
					var s stationReport
					s.epochTime = realTime.Unix() - baselineTime
					s.timeZone = station.timeZone
					s.latitude = station.latitude
					s.longitude = station.longitude
					s.altitude = station.altitude
					fmt.Printf("*** stationReport: %s\n", s)
					station.sendStationReport = false
					err := sendReport(n, cd, &station, leds, sync,
						[]byte(fmt.Sprintf("%s", s)),
						stationMessage)

					if err != nil {
						fmt.Printf("****sendStationReport error: %s\n", err)
						return // error will be ignored - in a goroutine
					}
					return
				}()
			}
			// flash led1 periodically
			if realTime.Unix()%flashInterval == 0 {
				alive <- emptyStruct{}
			}
		case <-windClick:
			windClicks++
		case <-rainClick:
			rainToday += rainMultiplier
			rainSinceLast += rainMultiplier
			rainHour[minutes] += rainMultiplier
		case <-sync:
			// resync the realTime variable to the clock
			t, err := clock.Read(&cd)
			if err != nil {
				fmt.Printf("failed to read clock: %s", err)
				// no point erroring we are in a go routine

			} else {
				realTime = t
			}
		case <-alive:
			if station.networkStatus == joinedLoraWan {
				leds <- aliveFlash
			} else {
				leds <- aliveNoNetwork
			}

			// never ends!
		case <-done:
			return
		}
	}
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

func formatStationReport(s stationReport) (string, error) {
	//fmt.Printf("lat: %02.5f lon: %02.5f alt: %d\n", s.latitude,
	//	s.longitude, s.altitude)
	o := format8Bytes(s.epochTime) +
		formatNbytes(float32(s.timeZone), 2) +
		formatLatLong(s.latitude*100000) +
		formatLatLong(s.longitude*100000) +
		formatInt16(s.altitude)
	return o, nil
}

// sendReport sends a slice of bytes to the interface.
// The slice should consist of hex characters - eg FF1234AB... - see
// formatStationReport and formatWeatherReport for the formatting
func sendReport(device *rak8nn.Device,
	clock ds3231.Device,
	sd *stationData,
	leds chan<- ledpanel.Control,
	sync chan<- emptyStruct,
	report []byte,
	reportType uint8) error {

	db, err := lorawan.Send(device, report, reportType)
	if err != nil {
		leds <- txError
		fmt.Printf("****tx error: %s\n", err)
		return statusErr{status: failedToSend,
			message: fmt.Sprintf("failed to send to lorawan: %s", err)}
	}
	leds <- txSuccess
	//fmt.Printf("datablock: %+v", db)

	if db == nil {
		fmt.Printf("****rx error: nil datablock - this should not happen\n")
		return nil
	}
	if db.Bytes == 0 {
		return nil
	}
	err = processDownload(db, sd, clock, leds, sync)
	if err != nil {
		return err // this will be ignored - in a goroutine
	}
	return nil
}

// ========================== Incoming messages =================================

// processdownload takes a pointer to a rak8nn.DataBlock and performs any
// actions required by the incoming message. The rak8nn.DataBlock is generated
// as a response to a loraWan send (see send Report). If there is a message
// queued up on the network, this will be loaded into the DataBlock. Even if
// there is no message, the network will respond with a DataBlock containing
// RSSI and SNR data.
//
// Incoming message types are:
// 200 - set timezone -
// 201 - set station data
// 202 - request station report
// 203 - reboot station
func processDownload(db *rak8nn.DataBlock,
	sd *stationData,
	cd ds3231.Device,
	leds chan<- ledpanel.Control,
	sync chan<- emptyStruct) error {

	switch db.Channel {
	case 200:
		if len(db.Data) < 4 {
			fmt.Printf("Incoming message: %d - invaild length: %d\n",
				db.Channel, len(db.Data))
			leds <- rxError
			return nil
		}
		sd.sendStationReport = true // Will send report during next cycle
		sd.timeZone = hex2int8(db.Data[0:2])

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
		sync <- emptyStruct{} // resync to t
		return nil
	case 201:
		if len(db.Data) < 20 {
			fmt.Printf("Incoming message: %d - invaild length: %d\n",
				db.Channel, len(db.Data))
			leds <- rxError
			return nil
		}
		sd.latitude = float32(hex2int32(db.Data[0:8])) / 100000 // five decimals
		sd.longitude = float32(hex2int32(db.Data[8:16])) / 100000
		sd.altitude = hex2int16(db.Data[16:20])
		sd.sendStationReport = true // Will send report during next cycle
		//fmt.Printf("Incoming message: %d , lat: %s, long: %s, alt: %s\n",
		//	db.Channel, db.Data[0:8], db.Data[8:16], db.Data[16:20])
		//fmt.Printf("Station Data: %s\n", s)
	case 202:
		// no data
		fmt.Printf("Incoming message: %d\n", db.Channel)
		sd.sendStationReport = true
	case 203:
		fmt.Printf("Incoming message: %d\n", db.Channel)
		time.Sleep(time.Second) // just to allow message time to get out
		rebootPin.High()
	default:
		// This is an error
		fmt.Printf("Incoming message: %d - unrecognised channel\n", db.Channel)
		leds <- rxError
	}
	return nil
}

//========================== Utilities and sensors =============================

// getWindDirection returns the angle of the weather vane in radians
// ( 2pi radians = 360 degrees)
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
	v := ((float32(b) - offset) / divisor)
	if v < 0 {
		v = 0
	}
	return v, nil
}

// windVectorAverage takes a slice of windVectors and calculates an average speed
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
// less likely to be confused about quadrants). The second parameter
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

// radiansToDegrees does exactly that
func radiansToDegrees(radians float32) uint16 {

	return uint16(math.Round(float64(radians * (180 / (pi)))))
}

// formatNbytes takes a float and attemps to return a string of n bytes of
// hex chars. EG formatNbytes(115, 2) returns "73"
// Does NOT do 2s complement conversions for -ve numbers
func formatNbytes(f float32, n uint8) string {
	f2 := int16(math.Round(float64(f))) // two implied decimals
	formatter := fmt.Sprintf("%%0%dx", n)
	return fmt.Sprintf(formatter, f2)
}

// formatInt16 takes an int16 and convert into a 4 char hex string with
// 2s complement eg -1 = FFFF
func formatInt16(i int16) string {
	if i >= 0 {
		return fmt.Sprintf("%04x", i)
	}
	// deal with negative number
	var i2 uint16
	i2 = uint16(0 - i) // make +ve
	i2 ^= 0xFFFF       // flip bits
	i2++               // Add 1 and hope it overflows...
	return fmt.Sprintf("%04x", i2)
}

// format8Bytes takes an int64 and outputs an 8 character hex string
// - no 2s complement conversion so avoid -ve numbers
func format8Bytes(i int64) string {
	return fmt.Sprintf("%08x", i)
}

// formatLatLong takes a float and outputs an eight character hex string,
// including sorting out a 2s copmplement representation.
// For example -10001 outputs FFFFD8EF
// Latitudes and longitudes are output with 5 implied decimal places, but that
// conversion is done by the calling routine.
func formatLatLong(l float32) string {
	if l >= 0 {
		l2 := int32(math.Round(float64(l)))
		return fmt.Sprintf("%08x", l2)
	}
	// we need to generate a 2s complement
	l3 := uint32(0 - l) // +ve version of the -ve no
	l3 ^= 0xFFFFFFFF    // flip all the bits
	l3++                // Add 1 and hope it overflows...
	return fmt.Sprintf("%08x", l3)

}

// formatWindDirection outputs a wind direction (in radians) as a three hex
// character string. eg  3.14 = 180 degrees, output as 0B4
func formatWindDirection(angle float32) string {
	d := radiansToDegrees(angle)
	return fmt.Sprintf("%03x", d)
}

// compensatePressure adjusts a barometric pressure to a sea-level equivalent
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

// hex2int8 takes 2 hex digits and convert to an int8, including handling
// 2s complement negatives
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

// getNum returns numeric value of a hex byte
// eg A returns 10, B 11 etc
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
