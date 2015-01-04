//#include <Arduino.h>


#define DHT11_WAKEUP 18
#define DHT11_TIMEOUT (F_CPU/40000)

unsigned char dht11_pin, dht11_bit;
volatile unsigned char *PIR;

void dht11_init(unsigned char pin)
{
	unsigned char port;
	dht11_pin = pin;
	dht11_bit = digitalPinToBitMask(pin);
	port = digitalPinToPort(pin);
	PIR = portInputRegister(port);
}

// data needs to be 5 bytes
bool dht11_read(unsigned char *data)
{
	unsigned char mask = 128, idx = 0, i;
	unsigned short count;
	unsigned long t;

	pinMode(dht11_pin, OUTPUT);
	digitalWrite(dht11_pin, LOW);
	delay(DHT11_WAKEUP);
    digitalWrite(dht11_pin, HIGH);
    delayMicroseconds(40);
    pinMode(dht11_pin, INPUT);

    count = DHT11_TIMEOUT;
    while ((*PIR & dht11_bit) == LOW) {
        if (--count == 0) return false;
    }

    count = DHT11_TIMEOUT;
    while ((*PIR & dht11_bit) != LOW) {
        if (--count == 0) return false;
    }

    for (i = 40; i != 0; i--) {
        count = DHT11_TIMEOUT;
        while ((*PIR & dht11_bit) == LOW) {
            if (--count == 0) return false;
        }

        t = micros();

        count = DHT11_TIMEOUT;
        while ((*PIR & dht11_bit) != LOW) {
            if (--count == 0) return false;
        }

        if ((micros() - t) > 40) 
            data[idx] |= mask;

        mask >>= 1;
        if (mask == 0) {
            mask = 128;
            idx++;
        }
    }
    pinMode(dht11_pin, OUTPUT);
    digitalWrite(dht11_pin, HIGH);

    return true;
}
