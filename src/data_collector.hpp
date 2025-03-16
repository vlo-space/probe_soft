#ifndef H_DATA_COLLECTOR
#define H_DATA_COLLECTOR

#include <cstdint>

class DataCollector {
  public:
    DataCollector() {}

    void addReading(double value, uint32_t time);
    double average() const;
    double averageSpeed() const;

    uint32_t timeSum() const { return this->time; }

    double valueSum() const { return this->value; }

  private:
    double lastReading = 0;
    uint32_t lastTime = 0;

    double value = 0;
    uint32_t readingCount = 0;
    uint32_t time = 0;
};

#endif