# APSC200
Team 13's APSC200 project. This uses Matlab's autonomous vehicle toolbox to simulate the detection and identification of parked vehicles.
## Running
To run the application, select a an existing scenario and run it. You will have to connect a database or remove references before running.
## Adding scenarios
To add a scenario, export from the DrivingScenarioDesigner and add a call to `process()` from the main loop. 
## Detection
Parked vehicles are detected and identified using a combination of LiDAR and cameras. LiDAR responses are processed in `LidarLib.m`, and then the camera is used to read extra data from the actors. 
## Database
The application is designed to connect to a database with the following schema:
| Field | Type        | Null | Key | Default           | Extra             |
|-------|-------------|------|-----|-------------------|-------------------|
| id    | int         | NO   | PRI | NULL              | auto_increment    |
| x     | int         | NO   |     | NULL              |                   |
| y     | int         | NO   |     | NULL              |                   |
| plate | varchar(16) | YES  |     | NULL              |                   |
| time  | datetime    | NO   |     | CURRENT_TIMESTAMP | DEFAULT_GENERATED |

Database features can be removed by removing all references to `conn` in `process.m`.
If you want to connect a database, connection must be established by `dbconn.m` (see `dbconn-default.m` for base code)