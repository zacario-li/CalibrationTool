#ifndef STORAGE_SERVICE_H
#define STORAGE_SERVICE_H

#include <string>
#include <vector>
#include <sqlite3.h> // SQLite C API
#include <filesystem> // For std::filesystem::remove if delete_if_exists

namespace core {
namespace common {

class StorageService {
public:
    StorageService();
    ~StorageService();

    // Opens a connection to the SQLite database.
    // If db_filepath is ":memory:", an in-memory database is used.
    // If delete_if_exists is true and the file exists, it will be deleted before opening.
    bool open(const std::string& db_filepath, bool delete_if_exists = false);

    // Closes the database connection.
    void close();

    // Creates a table if it doesn't exist.
    // table_name: Name of the table to create.
    // table_schema: SQL string defining the table columns and constraints (e.g., "id INTEGER PRIMARY KEY, name TEXT").
    bool create_table(const std::string& table_name, const std::string& table_schema);

    // Retrieves data from the database.
    // query: The full SQL SELECT query string.
    // Returns a vector of rows, where each row is a vector of strings (column values).
    // Returns an empty vector on error or if no data.
    std::vector<std::vector<std::string>> retrieve_data(const std::string& query);

    // Retrieves data including blobs. Blobs are returned as hex strings for now.
    // A more advanced version might return std::vector<std::variant<std::string, std::vector<unsigned char>>>
    // or have separate methods for blob retrieval.
    // For simplicity, this example will focus on text for retrieve_data.
    // A specific method to get a blob might be:
    // std::vector<unsigned char> retrieve_blob(const std::string& query, int blob_column_index);


    // Executes a general SQL statement (INSERT, UPDATE, DELETE, etc.) that does not return data rows.
    // statement_str: The SQL statement string.
    bool execute_statement(const std::string& statement_str);

    // Executes an SQL statement that includes binding a BLOB.
    // statement_str: SQL statement with a '?' placeholder for the blob (e.g., "INSERT INTO images (name, data) VALUES (?, ?)")
    // blob_data: The binary data to bind.
    // other_params: Optional vector of strings for other '?' placeholders, bound as text.
    //               The order of '?' in statement_str must match other_params then blob_data.
    //               This is a simplified example; a more robust solution would handle types.
    // Example usage assumes blob is the LAST parameter if other_params are used before it.
    // A better way is to use sqlite3_bind_parameter_count and bind by index carefully.
    // For this example: all '?' before the blob are text, the blob is the Nth '?'
    bool execute_statement_with_blob(
        const std::string& statement_str,
        const std::vector<std::string>& text_params, // Parameters to bind as text before the blob
        const std::vector<unsigned char>& blob_data,
        int blob_param_index // 1-based index of the blob parameter placeholder '?'
    );

    // Simpler version if blob is the only parameter or other params are hardcoded in statement
     bool execute_single_blob_statement(
        const std::string& statement_str, // SQL with one '?' for the blob
        const std::vector<unsigned char>& blob_data
    );


    bool is_open() const { return db_connection_ != nullptr; }
    std::string get_last_error_message() const;

private:
    sqlite3* db_connection_ = nullptr;
    std::string last_error_message_;

    // Static callback for sqlite3_exec (used by retrieve_data and potentially execute_statement)
    static int exec_callback(void* data, int argc, char** argv, char** azColName);
};

} // namespace common
} // namespace core

#endif // STORAGE_SERVICE_H
