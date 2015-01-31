var express = require('express');
var app = express();
var server = require('http').createServer(app);
var io = require('socket.io').listen(server);
var redis = require('redis');
var fs = require('fs');

app.get('/', function (req, res) {
  res.sendfile(__dirname + '/index.html');
});
app.use(express.static(__dirname + '/public'));

var redisClient = redis.createClient();

io.sockets.on('connection', function(socket) {
    console.log("a new user connected");
    socket.on('subscribe', function (data) {
        console.log("Subscribing user to new room: " + data);
        socket.join( data );
    });

    socket.on('unsubscribe', function (data) {
        console.log("Unsubscribing user from room: " + data);
        socket.leave( data );
    });

    socket.on('aircrafts', function(data) {
        console.log( "User requesting aircrafts" );
        
    });

	// when the user disconnects.. perform this
	socket.on('disconnect', function(){
        // ???
	});
});

// The server.ml sends periodic messages about all aircrafts.
// Instead of grabbing data from telemetry messages, which can be emitted very quickly,
// grab them from the periodic message send from server.ml.
// 
// NAV_STATUS, AP_STATUS, etc...
//
// The first element in the data is the ac_id.
//
var ground_subscriber = redis.createClient();
ground_subscriber.on("error", function (err) {
    console.log("error event - " + ground_subscriber.host + ":" + ground_subscriber.port + " - " + err);
});
ground_subscriber.on("pmessage", function( subscription, channel, data ) {
    var ac_id = data.split(" ")[0];
    var msgname = channel.split(".")[1];

    // console.log( "sending " + msgname + " about ac_id " + ac_id + ". data: " + data );

    io.sockets.in( ac_id ).emit( msgname, data );
});
ground_subscriber.psubscribe( "ground.*" );

server.listen( 3000 );

