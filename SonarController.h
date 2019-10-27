#ifndef SONAR_CONTROLLER_HH
#define SONAR_CONTROLLER_HH

const unsigned short int MAX_DIST = 23200;
const unsigned char SONARS_N = 5;

void _echo_isr();

class HC_SR04 {
  public:
    HC_SR04(unsigned char trigger, unsigned char echo) {
      _trigger = trigger;
      _echo = echo;
      range = -1;
      _finished = false;
    }

    void begin() {
      pinMode(_trigger, OUTPUT);
      digitalWrite(_trigger, LOW);
      pinMode(_echo, INPUT);
      attachPCINT(digitalPinToPCINT(_echo), _echo_isr, CHANGE);
      disablePCINT(digitalPinToPCINT(_echo));
    }

    void start() {
      _finished = false;
      enablePCINT(digitalPinToPCINT(_echo));
      digitalWrite(_trigger, HIGH);
      delayMicroseconds(10);
      digitalWrite(_trigger, LOW);
    }
    void stop() {
      _end = micros();
      _finished = true;
      getRange();
      disablePCINT(digitalPinToPCINT(_echo));
    }
    void getRange() {
      float dist = -1;
      if (_end - _start < MAX_DIST) dist = ((float)(_end - _start)) / 58;
      range = dist;
    }

    unsigned char _trigger, _echo;
    volatile float range;
    volatile unsigned long _start, _end;
    volatile bool _finished;
};

class SonarController {
  private:
    HC_SR04* sonars[SONARS_N];
    int i;
    bool isStart = false;
    volatile unsigned long timeOldSonars;
  public:
    SonarController(unsigned char TPLT, unsigned char EPLT, unsigned char TPRT, unsigned char EPRT, unsigned char TPCB, unsigned char EPCB, unsigned char TPLB, unsigned char EPLB, unsigned char TPRB, unsigned char EPRB) {
      i = 0;
      sonars[0] = new HC_SR04(TPRB, EPRB);
      sonars[1] = new HC_SR04(TPRT, EPRT);
      sonars[2] = new HC_SR04(TPCB, EPCB);
      sonars[3] = new HC_SR04(TPLT, EPLT);
      sonars[4] = new HC_SR04(TPLB, EPLB);
    }
    void init() {
      sonars[0]->begin();
      sonars[1]->begin();
      sonars[2]->begin();
      sonars[3]->begin();
      sonars[4]->begin();
      timeOldSonars = 0;
    }

    bool isReady() {
      return !isStart && (millis() - timeOldSonars >= 50);
    }

    void start() {
      sonars[i]->start();
      isStart = true;
    }

    HC_SR04* getCurrent() {
      return sonars[i];
    }

    void next() {
      i = (i + 1) % SONARS_N;
      timeOldSonars = millis();
      isStart = false;
    }

    void getRanges(float * res) {
      res[0] = sonars[0]->range;
      res[1] = sonars[1]->range;
      res[2] = sonars[2]->range;
      res[3] = sonars[3]->range;
      res[4] = sonars[4]->range;
    }
};

#endif
