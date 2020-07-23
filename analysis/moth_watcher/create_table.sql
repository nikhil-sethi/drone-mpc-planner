CREATE TABLE 'analytic_records'(
	'index' INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT UNIQUE, 
	'system' TEXT NOT NULL, 
	'flight_time' TEXT NOT NULL, 
	'flight_duration' REAL, 
	'start_RS_ID' INTEGER, 
	'velocity_mean' REAL, 
	'velocity_std' REAL, 
	'velocity_max' REAL, 
	'turning_angle_mean' REAL, 
	'turning_angle_std' REAL,
	'turning_angle_max' REAL, 
	'radial_accelaration_mean' REAL, 
	'radial_accelaration_std' REAL, 
	'radial_accelaration_max' REAL 
	UNIQUE('system','flight_time','flight_duration', 'start_RS_ID')
 );
