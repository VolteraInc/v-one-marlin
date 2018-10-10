#include "ToolBox.h"
#include "../../../serial.h"

static void outputToolUpdate(const char* type) {
  protocol
    << F("toolUpdate type:") << type
    << endl;
}

void tools::ToolBox::setTool(tools::Tool *tool) {
  if (m_currentTool == tool) {
    return;
  }

  if (m_currentTool) {
    m_currentTool->detach();
  }

  m_currentTool = tool;
  m_currentTool->attach();
  outputToolUpdate(m_currentTool->name());
}

tools::Tool& tools::ToolBox::determineTool(unsigned long time, float voltage, VoltageType& o_type) {
  // Check if current tool is still connected
  const auto isStillConnected = m_currentTool->checkIfConnected(
    m_voltageLog,
    time,
    voltage,
    &type
  );
  if (isStillConnected) {
    return *m_currentTool;
  }

  // Handle detaches
  // Note: a NullTool is used to represent the
  //       absense of an actual tool.
  if (m_currentTool != &nullTool) {
    return nullTool;
  }

  // Check for attach
  Tool* realTools[] = {
    &probe,
    &drill,
    &dispenser
  };
  for (auto tool : realTools) {
    const auto isConnected = m_currentTool->checkIfConnected(
      m_voltageLog,
      time,
      voltage,
      &type
    );

    // Tool deteected
    if (isConnected) {
      return *tool;
    }
  }

  type = UNKNOWN;
}

tools::Tool& tools::ToolBox::determineTool(unsigned long time, float voltage, VoltageType& type) {
  // Check if current tool is still connected
  // Note: type might be Unknown, and we don't want
  //       to treat that as a disconnect; doing so would
  //       make the system unnecessarily sensitive
  type = m_currentTool->classifyVoltage(
    m_voltageLog,
    time,
    voltage,
  );
  if (type != VoltageType::NoToolMounted) {
    return *m_currentTool;
  }

  // Handle detaches
  // Note: a NullTool is used to represent the
  //       absense of an actual tool.
  if (m_currentTool != &nullTool) {
    return nullTool;
  }

  // Check for attach
  Tool* realTools[] = {
    &probe,
    &drill,
    &dispenser
  };
  for (auto tool : realTools) {
    type = m_currentTool->classifyVoltage(
      m_voltageLog,
      time,
      voltage
    );

    // Tool deteected
    if (
      looksConnected
      // type != VoltageType::NoToolMounted &&
      // type != VoltageType::Unknown
      // type
    ) {
      return *tool;
    }
  }

  type = UNKNOWN;
  return *m_currentTool;
}


void tools::ToolBox::frequentInterruptibleWork() {
  // Note: Voltage climbs from 0 to 5 over ~60ms,
  //       Collecting every ~20ms should be sufficient
  //       for tool classification and detach detection.
  //       In the worst case it should take 3-4 samples.
  const auto now = millis();
  if (now < m_nextCheckAt) {
    return;
  }
  m_nextCheckAt = now + 20;

  // Grab the latest sample from the pTop pin
  const auto sample = m_pin.value();

  // Return if we already have this sample
  if (m_voltageLog.back().time == sample.startTime) {
    return;
  }

  // Determine the connected tool
  auto& detectedTool = determineTool(
    sample.startTime,
    sample.voltage,
    &type
  );

  // Change tools, unless detection is disabled
  if (m_toolDetectionEnabled) {
    setTool(&detectedTool);
  }

  // Record voltage and type
  m_voltages.push(sample.startTime, type, sample.voltage);
}

// TODO:
// Tools
//

//   if voltage is unstable/unkonwn for over x seconds, log
//   member fn must return 'type' and 'connected'
//     need Unknown to prevent disconnecting
//     need connected to prevent connecting when ProbeVoltageRising
//       could use tool's state...
//         only classify PVR and PMT if attached ... could work
//         ...then valid voltages like ProbeMounted would be withheld
//  ***          until we connect...might be a good thing.
//              call base to until stable, then classify?
//  *** expensive to search voltage log for length of stablity
//      (every time)... but does that really matter? (prob not)
//      might result in a better API.
//
//   - Tool::checkIfConnected
//      - stability thresholds
//      - Tool
//      - NullTool
//      - Probe,
//         - should we allow an intermittently triggered probe
//           to classify as a Probe?
//      - Dispenser
//      - Drill
//
//   - replace VoltageTypes with a const char* description
//
//   - VoltageLog
//     - move impl to own file
//     - populate log with some data to startTime
//       simplifies logic in classify
//
// Logging
//   - bonus - log channels
//       - log(tools), tools::log ?
//       - mLog, mLogDebug
//       - mLog, mLogDebug...tools::mLog => log << " Tools - "
//       - Confirm F("notice: ") does not allocate tons of memory
//   - fix logging from isr
//      - if in ISR add prefix + end
//   - logNotice or Warn if voltage Unknown for a while
//   - log on tool change (not stable)
//      - if type == NoToolMounted
//
