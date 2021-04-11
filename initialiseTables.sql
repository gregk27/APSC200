-- Create vehicles table
create table vehicles(
    id int not null key auto_increment, 
    x int not null, y int not null, 
    plate varchar(16), 
    time datetime not null default=CURRENT_TIMESTAMP);

-- Create areas table
create table areas (
    id int key auto_increment, 
    scenario varchar(32) not null, 
    name varchar(32) not null, 
    x0 int not null, 
    y0 int not null, 
    x1 int not null, 
    y1 int not null);

-- Create exemptions table
create table exemptions (
    id int key auto_increment
    plate varchar(16) not null);

-- Populate areas
insert into areas
            (scenario, name, x0, y0, x1, y1)
    values  ('city', 'Main St',    55,  25, -15,  15),
            ('city', 'Top St',     65,  35,  55, -15),
            ('city', 'Mid St',     25,  15,  15, -15),
            ('city', 'Bottom St',  -5,  15, -15, -15),
            ('test', 'test',       50,  50, -50, -50),

-- Populate exemptions
insert into exemptions
            (plate)
    values  ('657 UPRT'),
            ('QIQP 063'),
            ('800 UPYO')