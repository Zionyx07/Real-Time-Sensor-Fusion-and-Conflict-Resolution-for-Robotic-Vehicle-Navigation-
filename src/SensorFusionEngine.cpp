#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <unordered_set>
#include <memory>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <mutex>
#include <cstring>
#include <chrono>
#include <regex>

// Native POSIX Sockets (Zero external libraries)
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
enum class SensorType
{
    ULTRASONIC = 1,
    INFRARED = 2,
    LIDAR = 3
};

inline int getReliabilityScore(SensorType type)
{
    return static_cast<int>(type);
}

inline std::string sensorTypeToString(SensorType type)
{
    switch (type)
    {
    case SensorType::LIDAR:
        return "LIDAR";
    case SensorType::INFRARED:
        return "INFRARED";
    case SensorType::ULTRASONIC:
        return "ULTRASONIC";
    default:
        return "UNKNOWN";
    }
}

inline SensorType stringToSensorType(const std::string &str)
{
    std::string s = str;
    std::transform(s.begin(), s.end(), s.begin(), ::toupper);
    if (s == "LIDAR" || s == "2D_LIDAR")
        return SensorType::LIDAR;
    if (s == "IR" || s == "INFRARED")
        return SensorType::INFRARED;
    if (s == "ULTRASONIC" || s == "US")
        return SensorType::ULTRASONIC;
    throw std::invalid_argument("Unknown sensor type: " + str);
}

struct SensorEvent
{
    std::string event_id;
    SensorType sensor_type;
    std::string timestamp; // ISO 8601 UTC
    double value;
    int grid_x;
    int grid_y;

    // Strict deterministic total ordering: Timestamp ASC -> event_id ASC
    bool operator<(const SensorEvent &other) const
    {
        if (timestamp != other.timestamp)
        {
            return timestamp < other.timestamp;
        }
        return event_id < other.event_id;
    }
};

struct CellState
{
    int x;
    int y;
    double resolved_distance;
    SensorType winning_sensor;
    std::string last_updated;
    std::string last_winning_event_id;
};

struct AuditRecord
{
    long version;
    std::string incoming_event_id;
    std::string target_cell;
    std::string action_taken;
    std::string decision_reason;
    std::string processed_at;
};

struct EnvironmentSnapshot
{
    long version;
    std::string state_timestamp;
    std::map<std::string, CellState> grid; // Key: "x,y"
};

class SensorFusionEngine
{
private:
    std::unordered_set<std::string> processed_event_ids;
    std::vector<SensorEvent> ordered_event_history;
    std::map<long, EnvironmentSnapshot> snapshot_registry;
    std::vector<AuditRecord> audit_log;
    long version_counter = 0;
    std::map<std::string, CellState> current_grid;
    std::mutex engine_mutex;

    static std::string currentUtcTimestamp()
    {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        std::stringstream ss;
        ss << std::put_time(std::gmtime(&in_time_t), "%Y-%m-%dT%H:%M:%S")
           << '.' << std::setfill('0') << std::setw(3) << ms.count() << "Z";
        return ss.str();
    }

    void rebuildState()
    {
        current_grid.clear();
        std::map<std::string, SensorEvent> active_winners;

        for (const auto &ev : ordered_event_history)
        {
            std::string cell_key = std::to_string(ev.grid_x) + "," + std::to_string(ev.grid_y);
            auto it = active_winners.find(cell_key);

            if (it == active_winners.end())
            {
                active_winners[cell_key] = ev;
                current_grid[cell_key] = {
                    ev.grid_x, ev.grid_y, ev.value, ev.sensor_type, ev.timestamp, ev.event_id};
                audit_log.push_back({++version_counter,
                                     ev.event_id,
                                     cell_key,
                                     "INITIAL_OCCUPANCY",
                                     "First reading for grid cell",
                                     currentUtcTimestamp()});
            }
            else
            {
                SensorEvent current_winner = it->second;
                bool replace = false;
                std::string reason;

                int ev_score = getReliabilityScore(ev.sensor_type);
                int win_score = getReliabilityScore(current_winner.sensor_type);

                if (ev_score > win_score)
                {
                    replace = true;
                    reason = "Higher reliability: " + sensorTypeToString(ev.sensor_type) + " (" +
                             std::to_string(ev_score) + ") > " + sensorTypeToString(current_winner.sensor_type) +
                             " (" + std::to_string(win_score) + ")";
                }
                else if (ev_score < win_score)
                {
                    reason = "Rejected: Lower reliability " + sensorTypeToString(ev.sensor_type) + " < " +
                             sensorTypeToString(current_winner.sensor_type);
                }
                else
                {
                    // Equal reliability: compare timestamps
                    if (ev.timestamp > current_winner.timestamp)
                    {
                        replace = true;
                        reason = "Equal reliability; accepted newer timestamp: " + ev.timestamp;
                    }
                    else if (ev.timestamp < current_winner.timestamp)
                    {
                        reason = "Equal reliability; rejected older timestamp: " + ev.timestamp;
                    }
                    else
                    {
                        // Equal timestamp: tie-break via smallest event_id
                        if (ev.event_id < current_winner.event_id)
                        {
                            replace = true;
                            reason = "Tie-break: Smallest event_id: " + ev.event_id;
                        }
                        else
                        {
                            reason = "Tie-break: Larger event_id rejected: " + ev.event_id;
                        }
                    }
                }

                if (replace)
                {
                    active_winners[cell_key] = ev;
                    current_grid[cell_key] = {
                        ev.grid_x, ev.grid_y, ev.value, ev.sensor_type, ev.timestamp, ev.event_id};
                    audit_log.push_back({++version_counter,
                                         ev.event_id,
                                         cell_key,
                                         "CONFLICT_RESOLVED_OVERWRITE",
                                         reason,
                                         currentUtcTimestamp()});
                }
                else
                {
                    audit_log.push_back({++version_counter,
                                         ev.event_id,
                                         cell_key,
                                         "CONFLICT_REJECTED",
                                         reason,
                                         currentUtcTimestamp()});
                }
            }

            snapshot_registry[version_counter] = {
                version_counter,
                ev.timestamp,
                current_grid};
        }
    }

public:
    bool ingestEvent(const SensorEvent &ev)
    {
        std::lock_guard<std::mutex> lock(engine_mutex);

        if (processed_event_ids.find(ev.event_id) != processed_event_ids.end())
        {
            audit_log.push_back({version_counter,
                                 ev.event_id,
                                 std::to_string(ev.grid_x) + "," + std::to_string(ev.grid_y),
                                 "SKIPPED_DUPLICATE",
                                 "Event ID already processed; state unchanged",
                                 currentUtcTimestamp()});
            return false;
        }

        auto insert_pos = std::lower_bound(ordered_event_history.begin(), ordered_event_history.end(), ev);
        ordered_event_history.insert(insert_pos, ev);
        processed_event_ids.insert(ev.event_id);

        rebuildState();
        return true;
    }

    bool getSnapshot(long version, EnvironmentSnapshot &out_snapshot)
    {
        std::lock_guard<std::mutex> lock(engine_mutex);
        auto it = snapshot_registry.find(version);
        if (it != snapshot_registry.end())
        {
            out_snapshot = it->second;
            return true;
        }
        return false;
    }

    std::vector<AuditRecord> getAuditTrail()
    {
        std::lock_guard<std::mutex> lock(engine_mutex);
        return audit_log;
    }

    EnvironmentSnapshot replayToTimestamp(const std::string &up_to_time)
    {
        std::lock_guard<std::mutex> lock(engine_mutex);
        std::map<std::string, CellState> replay_grid;
        std::map<std::string, SensorEvent> active_winners;
        long replay_version = 0;

        for (const auto &ev : ordered_event_history)
        {
            if (ev.timestamp > up_to_time)
                break;
            replay_version++;
            std::string cell_key = std::to_string(ev.grid_x) + "," + std::to_string(ev.grid_y);
            auto it = active_winners.find(cell_key);

            if (it == active_winners.end())
            {
                active_winners[cell_key] = ev;
                replay_grid[cell_key] = {
                    ev.grid_x, ev.grid_y, ev.value, ev.sensor_type, ev.timestamp, ev.event_id};
            }
            else
            {
                bool replace = false;
                int ev_score = getReliabilityScore(ev.sensor_type);
                int win_score = getReliabilityScore(it->second.sensor_type);

                if (ev_score > win_score)
                {
                    replace = true;
                }
                else if (ev_score == win_score)
                {
                    if (ev.timestamp > it->second.timestamp)
                    {
                        replace = true;
                    }
                    else if (ev.timestamp == it->second.timestamp && ev.event_id < it->second.event_id)
                    {
                        replace = true;
                    }
                }

                if (replace)
                {
                    active_winners[cell_key] = ev;
                    replay_grid[cell_key] = {
                        ev.grid_x, ev.grid_y, ev.value, ev.sensor_type, ev.timestamp, ev.event_id};
                }
            }
        }
        return {replay_version, up_to_time, replay_grid};
    }
};

// Zero-dependency JSON Parsing & Formatting Helpers
inline std::string extractJsonString(const std::string &json, const std::string &key)
{
    std::regex re("\"" + key + "\"\\s*:\\s*\"([^\"]+)\"");
    std::smatch match;
    if (std::regex_search(json, match, re))
        return match[1].str();
    throw std::invalid_argument("Missing field: " + key);
}

inline double extractJsonDouble(const std::string &json, const std::string &key)
{
    std::regex re("\"" + key + "\"\\s*:\\s*([0-9.-]+)");
    std::smatch match;
    if (std::regex_search(json, match, re))
        return std::stod(match[1].str());
    throw std::invalid_argument("Missing double field: " + key);
}

inline int extractJsonNestedInt(const std::string &json, const std::string &parent, const std::string &child)
{
    std::regex re("\"" + parent + "\"\\s*:\\s*\\{[^}]*\"" + child + "\"\\s*:\\s*([0-9-]+)");
    std::smatch match;
    if (std::regex_search(json, match, re))
        return std::stoi(match[1].str());
    throw std::invalid_argument("Missing coordinate: " + child);
}

inline SensorEvent parseJsonEvent(const std::string &json)
{
    return {
        extractJsonString(json, "event_id"),
        stringToSensorType(extractJsonString(json, "sensor_type")),
        extractJsonString(json, "timestamp"),
        extractJsonDouble(json, "value"),
        extractJsonNestedInt(json, "location", "x"),
        extractJsonNestedInt(json, "location", "y")};
}

inline std::string formatSnapshotJson(const EnvironmentSnapshot &snap)
{
    std::stringstream ss;
    ss << "{\"version\":" << snap.version << ",\"timestamp\":\"" << snap.state_timestamp << "\",\"obstacles\":[";
    size_t count = 0;
    for (const auto &[key, cell] : snap.grid)
    {
        ss << "{\"x\":" << cell.x << ",\"y\":" << cell.y
           << ",\"distance\":" << std::fixed << std::setprecision(2) << cell.resolved_distance
           << ",\"sensor\":\"" << sensorTypeToString(cell.winning_sensor)
           << "\",\"eventId\":\"" << cell.last_winning_event_id << "\"}";
        if (++count < snap.grid.size())
            ss << ",";
    }
    ss << "]}";
    return ss.str();
}

inline std::string formatAuditJson(const std::vector<AuditRecord> &logs)
{
    std::stringstream ss;
    ss << "[";
    for (size_t i = 0; i < logs.size(); i++)
    {
        const auto &a = logs[i];
        ss << "{\"ver\":" << a.version << ",\"eventId\":\"" << a.incoming_event_id
           << "\",\"cell\":\"" << a.target_cell << "\",\"action\":\"" << a.action_taken
           << "\",\"reason\":\"" << a.decision_reason << "\",\"at\":\"" << a.processed_at << "\"}";
        if (i + 1 < logs.size())
            ss << ",";
    }
    ss << "]";
    return ss.str();
}

// Embedded POSIX HTTP Server
void startHttpServer(int port, SensorFusionEngine &engine)
{
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0)
    {
        perror("Socket creation failed");
        return;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
    {
        perror("Bind failed");
        close(server_fd);
        return;
    }

    if (listen(server_fd, 10) < 0)
    {
        perror("Listen failed");
        close(server_fd);
        return;
    }

    std::cout << ">> Sensor Fusion Engine C++ REST Server running on port " << port << std::endl;

    while (true)
    {
        int client_sock = accept(server_fd, nullptr, nullptr);
        if (client_sock < 0)
            continue;

        char buffer[4096] = {0};
        read(client_sock, buffer, sizeof(buffer) - 1);
        std::string request(buffer);

        std::string method, path;
        std::istringstream req_stream(request);
        req_stream >> method >> path;

        std::string status_code = "200 OK";
        std::string response_body;

        if (method == "POST" && path == "/events")
        {
            size_t body_pos = request.find("\r\n\r\n");
            if (body_pos != std::string::npos)
            {
                std::string body = request.substr(body_pos + 4);
                try
                {
                    SensorEvent ev = parseJsonEvent(body);
                    bool ingested = engine.ingestEvent(ev);
                    response_body = "{\"status\":\"" + std::string(ingested ? "ACCEPTED" : "DUPLICATE_IGNORED") +
                                    "\",\"eventId\":\"" + ev.event_id + "\"}";
                }
                catch (const std::exception &ex)
                {
                    status_code = "400 Bad Request";
                    response_body = "{\"error\":\"" + std::string(ex.what()) + "\"}";
                }
            }
        }
        else if (method == "GET" && path.rfind("/state/", 0) == 0)
        {
            try
            {
                long ver = std::stol(path.substr(7));
                EnvironmentSnapshot snap;
                if (engine.getSnapshot(ver, snap))
                {
                    response_body = formatSnapshotJson(snap);
                }
                else
                {
                    status_code = "404 Not Found";
                    response_body = "{\"error\":\"Version not found\"}";
                }
            }
            catch (...)
            {
                status_code = "400 Bad Request";
                response_body = "{\"error\":\"Invalid version\"}";
            }
        }
        else if (method == "GET" && path.rfind("/replay", 0) == 0)
        {
            size_t query_pos = path.find("until=");
            if (query_pos != std::string::npos)
            {
                std::string until_ts = path.substr(query_pos + 6);
                EnvironmentSnapshot snap = engine.replayToTimestamp(until_ts);
                response_body = formatSnapshotJson(snap);
            }
            else
            {
                status_code = "400 Bad Request";
                response_body = "{\"error\":\"Missing until parameter\"}";
            }
        }
        else if (method == "GET" && path == "/audit")
        {
            response_body = formatAuditJson(engine.getAuditTrail());
        }
        else
        {
            status_code = "404 Not Found";
            response_body = "{\"error\":\"Not Found\"}";
        }

        std::string http_response = "HTTP/1.1 " + status_code + "\r\n" + "Content-Type: application/json\r\n" + "Content-Length: " + std::to_string(response_body.length()) + "\r\n" + "Connection: close\r\n\r\n" + response_body;

        send(client_sock, http_response.c_str(), http_response.length(), 0);
        close(client_sock);
    }

    close(server_fd);
}

#ifndef ENGINE_TEST_BUILD
int main()
{
    SensorFusionEngine engine;
    startHttpServer(8080, engine);
    return 0;
}
#endif
