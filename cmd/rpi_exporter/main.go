package main

import (
	"flag"
	"log"
	"net/http"
	"os"

	"github.com/CHA0S-CORP/rpi_exporter/pkg/export/prometheus"
	"github.com/CHA0S-CORP/rpi_exporter/pkg/mbox"
	"github.com/CHA0S-CORP/rpi_exporter/pkg/sensehat"
)

var (
	flagAddr     = flag.String("addr", "", "Listen on address (e.g., :9110)")
	flagDebug    = flag.Bool("debug", false, "Print debug messages")
	flagSenseHat = flag.Bool("sensehat", false, "Enable Sense HAT metrics")
)

func main() {
	flag.Parse()
	mbox.Debug = *flagDebug

	// Initialize Sense HAT if enabled
	var hat *sensehat.SenseHat
	if *flagSenseHat {
		if !sensehat.Available() {
			log.Println("Warning: Sense HAT not detected, disabling Sense HAT metrics")
		} else {
			var err error
			hat, err = sensehat.New()
			if err != nil {
				log.Printf("Warning: Failed to initialize Sense HAT: %v", err)
			} else {
				log.Println("Sense HAT initialized")
				defer hat.Close()

				// Clear LED matrix
				if err := hat.ClearLEDs(); err != nil {
					log.Printf("Warning: Failed to clear LEDs: %v", err)
				}

				if hat.HasColorSensor() {
					log.Println("Color sensor detected (Sense HAT v2)")
				} else {
					log.Println("No color sensor detected (Sense HAT v1)")
				}
			}
		}
	}

	cfg := &prometheus.Config{
		SenseHat: hat,
	}

	if *flagAddr != "" {
		http.Handle("/metrics", http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			if err := prometheus.Write(w, cfg); err != nil {
				log.Printf("Error: %v", err)
				http.Error(w, http.StatusText(http.StatusInternalServerError), http.StatusInternalServerError)
			}
		}))
		log.Printf("Listening on %s", *flagAddr)
		if hat != nil {
			log.Println("Sense HAT metrics enabled")
		}
		if err := http.ListenAndServe(*flagAddr, nil); err != nil {
			log.Fatalf("Failed to start HTTP server: %v", err)
		}
		return
	}

	if err := prometheus.Write(os.Stdout, cfg); err != nil {
		log.Fatal(err)
	}
}