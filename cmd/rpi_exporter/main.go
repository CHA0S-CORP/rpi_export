package main

import (
	"bytes"
	"context"
	"encoding/json"
	"flag"
	"log"
	"net/http"
	"os"
	"os/signal"
	"syscall"
	"time"

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

				// Clear LED matrix on startup
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
		// Metrics endpoint
		http.Handle("/metrics", http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			// Flash LED on scrape
			if hat != nil {
				go hat.FlashLED(0, 0, 0, 255, 0, 100*time.Millisecond)
			}
			var buf bytes.Buffer
			if err := prometheus.Write(&buf, cfg); err != nil {
				log.Printf("Error: %v", err)
				http.Error(w, http.StatusText(http.StatusInternalServerError), http.StatusInternalServerError)
				return
			}
			w.Header().Set("Content-Type", "text/plain; version=0.0.4; charset=utf-8")
			buf.WriteTo(w)
		}))

		// Health check endpoint
		http.Handle("/health", http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			w.Header().Set("Content-Type", "text/plain")
			w.WriteHeader(http.StatusOK)
			w.Write([]byte("ok\n"))
		}))

		// Ready check endpoint
		http.Handle("/ready", http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			// Check if we can open the mailbox (VideoCore is accessible)
			m, err := mbox.Open()
			if err != nil {
				http.Error(w, "not ready: "+err.Error(), http.StatusServiceUnavailable)
				return
			}
			m.Close()
			w.Header().Set("Content-Type", "text/plain")
			w.WriteHeader(http.StatusOK)
			w.Write([]byte("ready\n"))
		}))

		// LED control endpoint
		http.Handle("/led", http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			if hat == nil {
				http.Error(w, "Sense HAT not available", http.StatusServiceUnavailable)
				return
			}

			switch r.Method {
			case http.MethodDelete:
				// Clear all LEDs
				if err := hat.ClearLEDs(); err != nil {
					http.Error(w, err.Error(), http.StatusInternalServerError)
					return
				}
				w.WriteHeader(http.StatusNoContent)

			case http.MethodPost:
				// Set pixel(s)
				var req struct {
					X int    `json:"x"`
					Y int    `json:"y"`
					R uint8  `json:"r"`
					G uint8  `json:"g"`
					B uint8  `json:"b"`
				}
				if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
					http.Error(w, "invalid JSON: "+err.Error(), http.StatusBadRequest)
					return
				}
				if err := hat.SetPixel(req.X, req.Y, req.R, req.G, req.B); err != nil {
					http.Error(w, err.Error(), http.StatusBadRequest)
					return
				}
				w.WriteHeader(http.StatusNoContent)

			default:
				http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
			}
		}))

		log.Printf("Listening on %s", *flagAddr)
		if hat != nil {
			log.Println("Sense HAT metrics enabled")
		}

		// Create server with timeouts
		srv := &http.Server{
			Addr:         *flagAddr,
			ReadTimeout:  10 * time.Second,
			WriteTimeout: 30 * time.Second,
			IdleTimeout:  60 * time.Second,
		}

		// Channel to listen for shutdown signals
		done := make(chan os.Signal, 1)
		signal.Notify(done, os.Interrupt, syscall.SIGINT, syscall.SIGTERM)

		// Start server in goroutine
		go func() {
			if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
				log.Fatalf("Failed to start HTTP server: %v", err)
			}
		}()

		// Wait for shutdown signal
		<-done
		log.Println("Shutting down...")

		// Turn off LEDs on signal
		if hat != nil {
			if err := hat.ClearLEDs(); err != nil {
				log.Printf("Warning: Failed to clear LEDs: %v", err)
			}
		}

		// Graceful shutdown with timeout
		ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
		defer cancel()

		if err := srv.Shutdown(ctx); err != nil {
			log.Fatalf("Server shutdown failed: %v", err)
		}
		log.Println("Server stopped")
		return
	}

	if err := prometheus.Write(os.Stdout, cfg); err != nil {
		log.Fatal(err)
	}
}
