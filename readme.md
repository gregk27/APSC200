# APSC200
Team 13's APSC200 project. This uses Matlab's autonomous vehicle toolbox to simulate the detection and identification of parked vehicles.
## Running
To run the application, select a an existing scenario and run it. You will have to connect a database before running.
## Adding scenarios
To add a scenario, export from the DrivingScenarioDesigner and add a call to `process()` from the main loop. 
## Detection
Parked vehicles are detected and identified using a combination of LiDAR and cameras. LiDAR responses are processed in `LidarLib.m`, and then the camera is used to read extra data from the actors. 

Vehicles will only be detected and recorded if their position is within a region from the areas table, and their plates are not listed in the exemptions table.

The complete detection process is outlined in the below flowchart.

![flowchart](https://imgur.com/Qmva8Kd.png)
## Database Preview
There is a node.js webserver application in `web/`, which when run will display database contents at `localhost:3000`. Database credentials must be in a `web/dbconfig.json` file, based on `dbconfig-default.json`.
## Output
When run, matlab will open 2 windows. The main window shows a top-down and 3rd-person veiw of the simulation. The second shows the lidar sensor's point cloud result. These are shown with the database preview below.
![image](https://imgur.com/BWPEzuH.png)
# Database
The application is designed to connect to a MySQL database with the following tables. `initialiseTables.sql` contains code to generate the tables and populate with data for the city scenario.
### vehicles
This table holds the list of detected vehicles.
| Field | Type        | Null | Key | Default           | Extra             |
|-------|-------------|------|-----|-------------------|-------------------|
| id    | int         | NO   | PRI | NULL              | auto_increment    |
| x     | int         | NO   |     | NULL              |                   |
| y     | int         | NO   |     | NULL              |                   |
| plate | varchar(16) | YES  |     | NULL              |                   |
| time  | datetime    | NO   |     | CURRENT_TIMESTAMP | DEFAULT_GENERATED |
 
 ### areas
 This table holds the list of areas where parking is illegal.

 Positions are in matlab world space, scenario is a tag passed into `process.m` used to prevent conflicts.
| Field    | Type        | Null | Key | Default | Extra          |
|----------|-------------|------|-----|---------|----------------|
| id       | int         | NO   | PRI | NULL    | auto_increment |
| scenario | varchar(32) | NO   |     | NULL    |                |
| name     | varchar(32) | NO   |     | NULL    |                |
| x0       | int         | NO   |     | NULL    |                |
| y0       | int         | NO   |     | NULL    |                |
| x1       | int         | NO   |     | NULL    |                |
| y1       | int         | NO   |     | NULL    |                |
 ### exemptions
 This table holds the list of exempt plates
| Field | Type        | Null | Key | Default | Extra          |
|-------|-------------|------|-----|---------|----------------|
| id    | int         | NO   | PRI | NULL    | auto_increment |
| plate | varchar(16) | NO   |     | NULL    |                |

Database features are required for operation. If you want to connect a database, connection must be established by a function `function conn = dbconn()` (see `dbconn-default.m` for base code)