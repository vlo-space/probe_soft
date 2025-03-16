#include "data_collector.hpp"

void DataCollector::addReading(double newValue, uint32_t newTime) {
    if (this->readingCount != 0) {
        this->value += newValue - this->lastReading;
        this->time += newTime - this->lastTime;
    }
    this->lastReading = newValue;
    this->lastTime = newTime;
    this->readingCount++;
}

double DataCollector::average() const {
    return this->value / (double) this->readingCount;
}

double DataCollector::averageSpeed() const {
    return this->value / (double) this->time;
}
