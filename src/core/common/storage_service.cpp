#include "storage_service.h"
#include <iostream> // For std::cerr
#include <vector>

namespace core {
namespace common {

StorageService::StorageService() : db_connection_(nullptr) {}

StorageService::~StorageService() {
    close();
}

bool StorageService::open(const std::string& db_filepath, bool delete_if_exists) {
    if (db_connection_) {
        close(); // Close any existing connection
    }

    if (delete_if_exists && db_filepath != ":memory:") {
        if (std::filesystem::exists(db_filepath)) {
            if (!std::filesystem::remove(db_filepath)) {
                last_error_message_ = "Failed to delete existing database file: " + db_filepath;
                std::cerr << "StorageService Error: " << last_error_message_ << std::endl;
                return false;
            }
        }
    }

    int rc = sqlite3_open(db_filepath.c_str(), &db_connection_);
    if (rc != SQLITE_OK) {
        last_error_message_ = "Cannot open database: " + std::string(sqlite3_errmsg(db_connection_));
        std::cerr << "StorageService Error: " << last_error_message_ << std::endl;
        sqlite3_close(db_connection_); // Close even if open failed to free resources
        db_connection_ = nullptr;
        return false;
    }
    last_error_message_.clear();
    return true;
}

void StorageService::close() {
    if (db_connection_) {
        sqlite3_close(db_connection_);
        db_connection_ = nullptr;
    }
}

bool StorageService::create_table(const std::string& table_name, const std::string& table_schema) {
    if (!db_connection_) {
        last_error_message_ = "Database not open.";
        std::cerr << "StorageService Error: " << last_error_message_ << std::endl;
        return false;
    }
    // Example schema: "id INTEGER PRIMARY KEY AUTOINCREMENT, name TEXT NOT NULL, data BLOB"
    std::string sql = "CREATE TABLE IF NOT EXISTS " + table_name + " (" + table_schema + ");";
    return execute_statement(sql);
}

// Static callback for sqlite3_exec
int StorageService::exec_callback(void* data, int argc, char** argv, char** azColName) {
    auto* rows = static_cast<std::vector<std::vector<std::string>>*>(data);
    std::vector<std::string> current_row;
    for (int i = 0; i < argc; i++) {
        current_row.push_back(argv[i] ? argv[i] : "NULL");
    }
    rows->push_back(current_row);
    return 0; // Continue execution
}

std::vector<std::vector<std::string>> StorageService::retrieve_data(const std::string& query) {
    std::vector<std::vector<std::string>> rows;
    if (!db_connection_) {
        last_error_message_ = "Database not open for retrieve_data.";
        std::cerr << "StorageService Error: " << last_error_message_ << std::endl;
        return rows; // Return empty
    }

    char* zErrMsg = nullptr;
    int rc = sqlite3_exec(db_connection_, query.c_str(), exec_callback, &rows, &zErrMsg);

    if (rc != SQLITE_OK) {
        last_error_message_ = "SQL error in retrieve_data: " + std::string(zErrMsg);
        std::cerr << "StorageService Error: " << last_error_message_ << std::endl;
        sqlite3_free(zErrMsg);
        rows.clear(); // Clear any partial results
    } else {
        last_error_message_.clear();
    }
    return rows;
}

bool StorageService::execute_statement(const std::string& statement_str) {
    if (!db_connection_) {
        last_error_message_ = "Database not open for execute_statement.";
        std::cerr << "StorageService Error: " << last_error_message_ << std::endl;
        return false;
    }

    char* zErrMsg = nullptr;
    int rc = sqlite3_exec(db_connection_, statement_str.c_str(), nullptr, 0, &zErrMsg);

    if (rc != SQLITE_OK) {
        last_error_message_ = "SQL error in execute_statement: " + std::string(zErrMsg);
        std::cerr << "StorageService Error: " << last_error_message_ << " (Statement: " << statement_str << ")" << std::endl;
        sqlite3_free(zErrMsg);
        return false;
    }
    last_error_message_.clear();
    return true;
}


bool StorageService::execute_statement_with_blob(
    const std::string& statement_str,
    const std::vector<std::string>& text_params,
    const std::vector<unsigned char>& blob_data,
    int blob_param_index) { // 1-based index

    if (!db_connection_) {
        last_error_message_ = "Database not open for execute_statement_with_blob.";
        std::cerr << "StorageService Error: " << last_error_message_ << std::endl;
        return false;
    }

    sqlite3_stmt* stmt = nullptr;
    int rc = sqlite3_prepare_v2(db_connection_, statement_str.c_str(), -1, &stmt, nullptr);

    if (rc != SQLITE_OK) {
        last_error_message_ = "Failed to prepare statement: " + std::string(sqlite3_errmsg(db_connection_));
        std::cerr << "StorageService Error: " << last_error_message_ << " (Statement: " << statement_str << ")" << std::endl;
        return false;
    }

    // Bind text parameters
    for (size_t i = 0; i < text_params.size(); ++i) {
        // Assuming text params are bound before the blob param index
        // This logic needs to be robust if blob is not the last or among text params.
        // For now, assume text_params correspond to the first N '?' and blob is at blob_param_index.
        int current_param_idx = static_cast<int>(i + 1);
        if (current_param_idx >= blob_param_index) { // Shift text param index if it's after where blob will be
            current_param_idx++;
        }
         rc = sqlite3_bind_text(stmt, current_param_idx, text_params[i].c_str(), -1, SQLITE_STATIC);
        if (rc != SQLITE_OK) {
            last_error_message_ = "Failed to bind text parameter " + std::to_string(i+1) + ": " + std::string(sqlite3_errmsg(db_connection_));
            std::cerr << "StorageService Error: " << last_error_message_ << std::endl;
            sqlite3_finalize(stmt);
            return false;
        }
    }

    // Bind blob data
    // sqlite3_bind_blob takes pointer to data, size in bytes, and a destructor type (SQLITE_STATIC means data must exist until step)
    // SQLITE_TRANSIENT makes a copy.
    rc = sqlite3_bind_blob(stmt, blob_param_index, blob_data.data(), static_cast<int>(blob_data.size()), SQLITE_TRANSIENT);
    if (rc != SQLITE_OK) {
        last_error_message_ = "Failed to bind blob parameter: " + std::string(sqlite3_errmsg(db_connection_));
        std::cerr << "StorageService Error: " << last_error_message_ << std::endl;
        sqlite3_finalize(stmt);
        return false;
    }

    rc = sqlite3_step(stmt);
    if (rc != SQLITE_DONE) {
        last_error_message_ = "Execution failed: " + std::string(sqlite3_errmsg(db_connection_));
        std::cerr << "StorageService Error: " << last_error_message_ << std::endl;
        sqlite3_finalize(stmt);
        return false;
    }

    sqlite3_finalize(stmt);
    last_error_message_.clear();
    return true;
}

bool StorageService::execute_single_blob_statement(
    const std::string& statement_str, // SQL with one '?' for the blob
    const std::vector<unsigned char>& blob_data) {

    if (!db_connection_) {
        last_error_message_ = "Database not open for execute_single_blob_statement.";
        std::cerr << "StorageService Error: " << last_error_message_ << std::endl;
        return false;
    }

    sqlite3_stmt* stmt = nullptr;
    int rc = sqlite3_prepare_v2(db_connection_, statement_str.c_str(), -1, &stmt, nullptr);

    if (rc != SQLITE_OK) {
        last_error_message_ = "Failed to prepare statement: " + std::string(sqlite3_errmsg(db_connection_));
        std::cerr << "StorageService Error: " << last_error_message_ << " (Statement: " << statement_str << ")" << std::endl;
        return false;
    }

    rc = sqlite3_bind_blob(stmt, 1, blob_data.data(), static_cast<int>(blob_data.size()), SQLITE_TRANSIENT);
    if (rc != SQLITE_OK) {
        last_error_message_ = "Failed to bind blob parameter: " + std::string(sqlite3_errmsg(db_connection_));
        std::cerr << "StorageService Error: " << last_error_message_ << std::endl;
        sqlite3_finalize(stmt);
        return false;
    }

    rc = sqlite3_step(stmt);
    if (rc != SQLITE_DONE) {
        last_error_message_ = "Execution failed: " + std::string(sqlite3_errmsg(db_connection_));
        std::cerr << "StorageService Error: " << last_error_message_ << std::endl;
        sqlite3_finalize(stmt);
        return false;
    }

    sqlite3_finalize(stmt);
    last_error_message_.clear();
    return true;
}


std::string StorageService::get_last_error_message() const {
    return last_error_message_;
}


} // namespace common
} // namespace core
