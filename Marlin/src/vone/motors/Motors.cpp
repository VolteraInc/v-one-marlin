#include "Motors.h"

#include "../../../serial.h"

static const __FlashStringHelper* s_motorStatusToString(bool isTriggered) {
  return isTriggered ? F(" on") : F(" off");
}

void Motors::outputStatus() const {
  const auto sp = F("  ");
  log << F("Motor status") << endl;
  log << sp << xAxis.name << s_motorStatusToString(xAxis.enabled()) << endl;
  log << sp << yAxis.name << s_motorStatusToString(xAxis.enabled()) << endl;
  log << sp << zAxis.name << s_motorStatusToString(xAxis.enabled()) << endl;
  log << sp << eAxis.name << s_motorStatusToString(xAxis.enabled()) << endl;
}


static void s_reportAndUpdateStatus(
  const __FlashStringHelper* name,
  bool& reportedStatus,
  bool currentStatus
) {
  if (reportedStatus != currentStatus) {
    reportedStatus = currentStatus;
    log << name << F(" motor was turned ") << s_motorStatusToString(currentStatus) << endl;
  }
}

void Motors::reportChanges() {
  s_reportAndUpdateStatus(xAxis.name, m_reportedStatus.xAxisEnabled, xAxis.enabled());
  s_reportAndUpdateStatus(yAxis.name, m_reportedStatus.yAxisEnabled, yAxis.enabled());
  s_reportAndUpdateStatus(zAxis.name, m_reportedStatus.zAxisEnabled, zAxis.enabled());
  s_reportAndUpdateStatus(eAxis.name, m_reportedStatus.eAxisEnabled, eAxis.enabled());
}