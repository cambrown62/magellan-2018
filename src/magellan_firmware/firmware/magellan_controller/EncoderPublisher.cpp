#include "EncoderPublisher.h"
#include <Arduino.h>
#include "config.h"

static bool encoder_isr_configured = false;
volatile static long int left_encoder_count = 0;
volatile static long int right_encoder_count = 0;

static void LeftEncoderISR() {
    left_encoder_count++;
}

static void RightEncoderISR() {
    right_encoder_count++;
}

static void SetupEncoderISR() {
    if ( encoder_isr_configured )
        return;
    encoder_isr_configured = true;

    pinMode(LEFT_ENCODER, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER), LeftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER), RightEncoderISR, CHANGE);
}


EncoderPublisher::EncoderPublisher(ros::NodeHandle& nh) :
        nh_(nh),
        encoder_msg_(),
        encoder_publisher_("/platform/encoders", &encoder_msg_),
        update_rate_(ENCODER_UPDATE_HZ) {
    SetupEncoderISR();

    nh.advertise(encoder_publisher_);
}

void EncoderPublisher::Update(bool forward) {
    if ( update_rate_.NeedsRun() ) {
        // Atomic region
        cli();
        long int left_delta = left_encoder_count;
        long int right_delta = right_encoder_count;

        left_encoder_count = 0;
        right_encoder_count = 0;
        sei();

        if (!forward) {
            left_delta *= -1;
            right_delta *= -1;
        }

        encoder_msg_.header.stamp = nh_.now();
        encoder_msg_.left_delta = left_delta;
        encoder_msg_.right_delta = right_delta;

        encoder_publisher_.publish(&encoder_msg_);
    }
}
