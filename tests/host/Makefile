# Host-side tests for the robot firmware's portable modules.
# Pure C, no kernel or hardware: builds and runs on the development machine.
#
#   make test     build and run every test
#   make sim      print the simulated barcode decode-rate table
#   make clean
#
# To add a test: create test_<name>.c, list it in TESTS and name the module
# sources it links in SRC_test_<name>.

CC      ?= cc
APP     := ../../app_program
CFLAGS  := -std=c11 -O1 -g -Wall -Wextra -Wpedantic -Wconversion -Wshadow \
           -Werror -I$(APP) -I.
LDLIBS  := -lm

TESTS   := test_barcode test_barcode_map test_line_sensor test_pipeline test_fault_mask \
           test_encoder test_pid test_motion test_motor_cal test_imu test_ultrasonic test_obstacle test_line_nav test_telemetry test_mqtt_codec test_mqtt_client test_safety test_vehicle test_follow_sim

SRC_test_barcode      := barcode.c
SRC_test_barcode_map  := barcode_map.c
SRC_test_line_sensor  := line_sensor.c
SRC_test_pipeline     := barcode.c line_sensor.c
SRC_test_fault_mask   := fault_mask.c
SRC_test_encoder      := encoder.c
SRC_test_pid          := pid.c
SRC_test_motion       := motion.c pid.c encoder.c
SRC_test_motor_cal    := motor_cal.c
SRC_test_imu          := imu.c
SRC_test_ultrasonic   := ultrasonic.c servo.c
SRC_test_obstacle     := obstacle.c ultrasonic.c servo.c
SRC_test_line_nav     := line_nav.c line_sensor.c barcode.c
SRC_test_telemetry    := telemetry.c
SRC_test_mqtt_codec   := mqtt_codec.c
SRC_test_mqtt_client  := mqtt_client.c mqtt_codec.c
SRC_test_safety       := safety.c fault_mask.c
SRC_test_follow_sim   := line_nav.c line_sensor.c barcode.c
SRC_test_vehicle      := vehicle.c line_nav.c line_sensor.c barcode.c obstacle.c ultrasonic.c servo.c motion.c pid.c encoder.c

.PHONY: test clean sim stress
test: $(TESTS)
	@for t in $(TESTS); do echo "== $$t"; ./$$t || exit 1; done

.SECONDEXPANSION:
$(TESTS): %: %.c test_support.h barcode_fixtures.h $$(wildcard $(APP)/*.h) \
          $$(addprefix $(APP)/,$$(SRC_$$@))
	$(CC) $(CFLAGS) -o $@ $< $(addprefix $(APP)/,$(SRC_$@)) $(LDLIBS)

# Not a test: prints decode rates per sample period and speed (simulation).
sim_report: sim_report.c sim_trace.h $(APP)/barcode.c $(APP)/line_sensor.c \
            barcode_fixtures.h
	$(CC) $(CFLAGS) -o $@ sim_report.c $(APP)/barcode.c $(APP)/line_sensor.c

sim: sim_report
	./sim_report

# Not a test of the default run: the fault-mask concurrency stress test (needs pthreads) and its negative control.
# make stress   -> the protected build must PASS and the -DUNPROTECTED control must lose updates.
stress: ../../app_program/fault_mask.c test_fault_mask_stress.c
	$(CC) -std=c11 -O2 -Wall -Wextra -pthread -I$(APP) -o stress_protected test_fault_mask_stress.c $(APP)/fault_mask.c
	$(CC) -std=c11 -O2 -Wall -Wextra -pthread -DUNPROTECTED -I$(APP) -o stress_unprotected test_fault_mask_stress.c $(APP)/fault_mask.c
	./stress_protected
	./stress_unprotected

clean:
	rm -f $(TESTS) sim_report stress_protected stress_unprotected
