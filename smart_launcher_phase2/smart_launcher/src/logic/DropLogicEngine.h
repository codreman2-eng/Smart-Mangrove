#pragma once
#include <Arduino.h>
#include "../config/ConfigManager.h"
#include "../position/IPositionSource.h"
#include "../actuator/ActuatorModule.h"
#include "../safety/StateMachine.h"
#include "../logging/Logger.h"

class DropLogicEngine {
public:
    DropLogicEngine(ConfigManager&   cfg,
                    IPositionSource& pos,
                    ActuatorModule&  modA,
                    ActuatorModule&  modB,
                    StateMachine&    sm,
                    Logger&          log);

    // Call every loop iteration — evaluates drop conditions
    void update();

    // True when all 10 drops are complete
    bool missionComplete() const;

private:
    ConfigManager&   _cfg;
    IPositionSource& _pos;
    ActuatorModule&  _modA;
    ActuatorModule&  _modB;
    StateMachine&    _sm;
    Logger&          _log;

    uint8_t _next_drop_idx = 0;   // index into drops[] array

    // Execute one synchronized drop: A first, then B after A confirms
    void _executeDrop(uint8_t drop_idx);

    // Build and log a DropEvent
    void _recordDrop(uint8_t drop_id, uint8_t module_idx,
                     uint8_t door_num, const Position& pos,
                     DropResult result);
};
