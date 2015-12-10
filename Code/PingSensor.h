class pingSensor
{

    int _pingPin;
    unsigned long _distCm;
    unsigned long _distInch;
    unsigned long _duration;

  public:

    pingSensor(int pingPin)
    {
      _pingPin = pingPin;
    }

    long getPingCm() {
      // establish variables for duration of the ping,
      // and the distance result in inches and centimeters:
      //  long duration, inches, cm;

      // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
      // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
      pinMode(_pingPin, OUTPUT);
      digitalWrite(_pingPin, LOW);
      delayMicroseconds(2);
      digitalWrite(_pingPin, HIGH);
      delayMicroseconds(5);
      digitalWrite(_pingPin, LOW);

      // The same pin is used to read the signal from the PING))): a HIGH
      // pulse whose duration is the time (in microseconds) from the sending
      // of the ping to the reception of its echo off of an object.
      pinMode(_pingPin, INPUT);
      _duration = pulseIn(_pingPin, HIGH);

      // The speed of sound is 340 m/s or 29 microseconds per centimeter.
      // The ping travels out and back, so to find the distance of the
      // object we take half of the distance travelled.

      _distCm = _duration / 29 / 2;
      return _distCm;
    }

      long getPingInch() {
      // establish variables for duration of the ping,
      // and the distance result in inches and centimeters:
      //  long duration, inches, cm;

      // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
      // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
      pinMode(_pingPin, OUTPUT);
      digitalWrite(_pingPin, LOW);
      delayMicroseconds(2);
      digitalWrite(_pingPin, HIGH);
      delayMicroseconds(5);
      digitalWrite(_pingPin, LOW);

      // The same pin is used to read the signal from the PING))): a HIGH
      // pulse whose duration is the time (in microseconds) from the sending
      // of the ping to the reception of its echo off of an object.
      pinMode(_pingPin, INPUT);
      _duration = pulseIn(_pingPin, HIGH);

     // According to Parallax's datasheet for the PING))), there are
      // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
      // second).  This gives the distance travelled by the ping, outbound
      // and return, so we divide by 2 to get the distance of the obstacle.
      // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
      
      _distInch = _duration / 74 / 2;
      return _distInch;
    }
};
