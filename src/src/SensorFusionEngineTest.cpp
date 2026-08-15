#define ENGINE_TEST_BUILD
#include "SensorFusionEngine.cpp"
#include <cassert>
#include <iostream>

void testIdempotency() {
    SensorFusionEngine engine;
    SensorEvent ev1{"EVT_DUP_001", SensorType::ULTRASONIC, "2026-08-15T10:00:00.000Z", 1.50, 5, 5};

    assert(engine.ingestEvent(ev1) == true);
    assert(engine.ingestEvent(ev1) == false); // Duplicate rejected

    EnvironmentSnapshot snap;
    assert(engine.getSnapshot(1, snap) == true);
    assert(snap.grid.size() == 1);
    std::cout << "[PASS] Idempotency & Duplicate Rejection\n";
}

void testReliabilityHierarchy() {
    SensorFusionEngine engine;
    SensorEvent us_ev{"EVT_US", SensorType::ULTRASONIC, "2026-08-15T10:00:01.000Z", 3.0, 2, 2};
    SensorEvent lidar_ev{"EVT_LIDAR", SensorType::LIDAR, "2026-08-15T10:00:02.000Z", 1.25, 2, 2};

    engine.ingestEvent(us_ev);
    engine.ingestEvent(lidar_ev);

    EnvironmentSnapshot snap;
    assert(engine.getSnapshot(2, snap) == true);
    assert(snap.grid.at("2,2").winning_sensor == SensorType::LIDAR);
    assert(snap.grid.at("2,2").resolved_distance == 1.25);
    std::cout << "[PASS] Reliability Priority (LIDAR > IR > Ultrasonic)\n";
}

void testTimestampArbitration() {
    SensorFusionEngine engine;
    SensorEvent ir_old{"EVT_T1", SensorType::INFRARED, "2026-08-15T10:00:01.000Z", 2.50, 4, 4};
    SensorEvent ir_new{"EVT_T2", SensorType::INFRARED, "2026-08-15T10:00:05.000Z", 1.80, 4, 4};

    engine.ingestEvent(ir_old);
    engine.ingestEvent(ir_new);

    EnvironmentSnapshot snap;
    assert(engine.getSnapshot(2, snap) == true);
    assert(snap.grid.at("4,4").last_winning_event_id == "EVT_T2");
    assert(snap.grid.at("4,4").resolved_distance == 1.80);
    std::cout << "[PASS] Timestamp Arbitration\n";
}

void testDeterministicTieBreaking() {
    SensorFusionEngine engine;
    std::string timestamp = "2026-08-15T10:00:00.000Z";
    SensorEvent ev_b{"EVT_B", SensorType::LIDAR, timestamp, 2.0, 7, 7};
    SensorEvent ev_a{"EVT_A", SensorType::LIDAR, timestamp, 1.1, 7, 7};

    engine.ingestEvent(ev_b);
    engine.ingestEvent(ev_a);

    EnvironmentSnapshot snap;
    assert(engine.getSnapshot(2, snap) == true);
    assert(snap.grid.at("7,7").last_winning_event_id == "EVT_A");
    assert(snap.grid.at("7,7").resolved_distance == 1.1);
    std::cout << "[PASS] Deterministic Tie-Breaking\n";
}

void testLateEventReordering() {
    SensorFusionEngine engine;
    SensorEvent late_us{"EVT_LATE_US", SensorType::ULTRASONIC, "2026-08-15T10:00:10.000Z", 5.0, 8, 8};
    SensorEvent early_lidar{"EVT_EARLY_LIDAR", SensorType::LIDAR, "2026-08-15T10:00:05.000Z", 1.2, 8, 8};

    engine.ingestEvent(late_us);      // Ingested first
    engine.ingestEvent(early_lidar);  // Out of order late arrival

    EnvironmentSnapshot snap;
    assert(engine.getSnapshot(2, snap) == true);
    assert(snap.grid.at("8,8").winning_sensor == SensorType::LIDAR);
    std::cout << "[PASS] Out-of-Order / Late Event Resolution\n";
}

void testHistoricalStateReplay() {
    SensorFusionEngine engine;
    engine.ingestEvent({"E1", SensorType::LIDAR, "2026-08-15T10:00:00.000Z", 1.0, 1, 1});
    engine.ingestEvent({"E2", SensorType::LIDAR, "2026-08-15T10:05:00.000Z", 2.0, 1, 1});
    engine.ingestEvent({"E3", SensorType::LIDAR, "2026-08-15T10:10:00.000Z", 3.0, 1, 1});

    EnvironmentSnapshot historic = engine.replayToTimestamp("2026-08-15T10:02:00.000Z");
    assert(historic.grid.at("1,1").last_winning_event_id == "E1");
    assert(historic.grid.at("1,1").resolved_distance == 1.0);
    std::cout << "[PASS] Historical State Replay\n";
}

int main() {
    std::cout << "=== Starting C++ Sensor Fusion Engine Test Suite ===\n";
    testIdempotency();
    testReliabilityHierarchy();
    testTimestampArbitration();
    testDeterministicTieBreaking();
    testLateEventReordering();
    testHistoricalStateReplay();
    std::cout << "=== ALL C++ TESTS PASSED SUCCESSFULLY ===\n";
    return 0;
}
