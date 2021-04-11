const express = require('express');
const mysql = require('mysql');
const CONFIG = require('./dbcfg.json');
const app = express();
const port = 3000;

var conn = mysql.createConnection(CONFIG);

conn.connect();

app.get('/', (req, res) => {
    res.sendFile(__dirname+"/index.html");
});

app.get('/data/vehicles', (req, res) => {
    conn.query("select * from vehicles order by id desc", function(error, results, field) {
        if(error) throw error;
        res.json(results);
    });
});
app.get('/data/areas/:scenario', (req, res) => {
    conn.query(`select name,x0,y0,x1,y1 from areas where scenario='${req.params.scenario}'`, function(error, results, field) {
        if(error) throw error;
        res.json(results);
    });
});
app.get('/data/exempt', (req, res) => {
    conn.query(`select plate from exemptions`, function(error, results, field) {
        if(error) throw error;
        res.json(results);
    });
});

app.listen(port, () => {
    console.log("Listening on port " + port);
})