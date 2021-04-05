function [conn] = dbconn()
    vendor = "MySQL";
    src = databaseConnectionOptions("jdbc",vendor);

    src = setoptions(src, ...
    'DataSourceName',"MySQL", ...
    'JDBCDriverLocation',"C:/Program Files/MySQL/MySQL Connector J/mysql-connector-java-8.0.23.jar", ...
    'DatabaseName',"[DB]",'Server',"[SERVER]", ...
    'PortNumber',3306,'AuthenticationType',"Server");

    testConnection(src, "[USER]", "[PASS]")
    saveAsDataSource(src);

    conn = database("MySQL", "[USER]", "[PASS]");
end