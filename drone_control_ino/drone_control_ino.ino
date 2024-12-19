// Drone States and Modes
enum DroneState {
  START_MISSION,
  HIGH_ALTITUDE_SWEEP,
  STEALTH_SWEEP
};

enum MissionMode {
  UNDEFINED,
  SEARCH_RESCUE,
  RECONNAISSANCE
};

class DroneMission {
  private:
    DroneState currentState;
    MissionMode selectedMode;
    bool modeConfirmed;
    
  public:
    DroneMission() {
      currentState = START_MISSION;
      selectedMode = UNDEFINED;
      modeConfirmed = false;
    }

    void handleCommand(String command) {
      if (currentState == START_MISSION) {
        if (!modeConfirmed) {
          // Mode Selection
          if (command == "UP") {
            selectedMode = SEARCH_RESCUE;
            Serial.println("Search & Rescue mode selected. Make STOP gesture to confirm.");
          }
          else if (command == "DOWN") {
            selectedMode = RECONNAISSANCE;
            Serial.println("Reconnaissance mode selected. Make STOP gesture to confirm.");
          }
          // Mode Confirmation
          else if (command == "STOP" && selectedMode != UNDEFINED) {
            modeConfirmed = true;
            Serial.print("Mode confirmed: ");
            Serial.println(selectedMode == SEARCH_RESCUE ? "Search & Rescue" : "Reconnaissance");
            Serial.println("Running system checks...");
            if (runSystemChecks()) {
              currentState = selectedMode == SEARCH_RESCUE ? HIGH_ALTITUDE_SWEEP : STEALTH_SWEEP;
              Serial.println("System checks complete. Moving to initial sweep.");
            }
          }
          // Allow mode change with LEFT gesture before confirmation
          else if (command == "LEFT" && !modeConfirmed) {
            selectedMode = UNDEFINED;
            Serial.println("Mode selection reset. Please select mode:");
            Serial.println("UP = Search & Rescue");
            Serial.println("DOWN = Reconnaissance");
          }
        }
      }
    }

    bool runSystemChecks() {
      Serial.println("1. Checking battery level...");
      delay(500);  // Simulate check time
      
      Serial.println("2. Checking motors...");
      delay(500);
      
      Serial.println("3. Checking sensors...");
      delay(500);
      
      Serial.println("4. Checking communication systems...");
      delay(500);
      
      Serial.println("All systems GO!");
      return true;  // Return false if any check fails
    }
};

// Global mission object
DroneMission mission;

void setup() {
  Serial.begin(9600);
  Serial.println("Drone initialization starting...");
  Serial.println("Please select mode:");
  Serial.println("UP = Search & Rescue");
  Serial.println("DOWN = Reconnaissance");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    mission.handleCommand(command);
  }
}