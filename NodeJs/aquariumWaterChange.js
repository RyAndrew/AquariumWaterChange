"use strict";

const http = require('http');
const net = require('net');
const url = require('url');
const WebSocket = require('ws');
const fs = require('fs');
const path = require('path');
const querystring = require('querystring');

const SerialPort = require('serialport');
const Readline = require('@serialport/parser-readline')

const arduinoCommands = {
	PING:'a',

	DRAIN_TANK:'b', //opens solenoid valve until tank low sensor triggers
	FILL_TANK:'c',//checks tank low is triggered and barrel full is triggered, runs pump until tank high is triggered or barrel low is triggered

	DRAIN_TANK_VALVE_OPEN: 'd',
	DRAIN_TANK_VALVE_CLOSE: 'e',

	FILL_TANK_PUMP_ON: 'f',
	FILL_TANK_PUMP_OFF: 'g',

	RODI_AIR_HEAT_ON: 'h',
	RODI_AIR_HEAT_OFF: 'i',

	RESCAN_TEMP_PROBES: 'j',
};

const validStates = [
	'm1',
	'r1',
	'r2',
	't1',
	't2',
	'w1',
	'w2',
	'w3',
	'w4',
	'w1'
];


var arduinoSerial;
var arduinoState = {};

validStates.forEach(v => {
	arduinoState[v] = '-1'
});

var config = {
	httpPort:80 //https://stackoverflow.com/a/23281401
};

function init(){
	initSerial();
	initHttpServer();
	console.log('http init');
	//setInterval(intervalFunc, 1500);
}

function initSerial() {

	arduinoSerial = new SerialPort('/dev/ttyAMA0', {
		baudRate: 115200
	}
	);

	arduinoSerial.on('error', function(err) {
		console.log('arduinoSerial Error: ', err.message)
	})

	const parser = arduinoSerial.pipe(new Readline({ delimiter: '\r\n' }))
	parser.on('data', handleSerialMessage)

}

function handleSerialMessage(data){
	let keyValueArr = data.split(",");
	let stateChangeFound = false;
	keyValueArr.forEach(entry => {
		let key, value;
		[key,value] = entry.split("=");

		if(!validStates.includes(key)){
			return;
		}
		//console.log(key,'=',value);
		if(arduinoState[key] !== value ){
			stateChangeFound = true;
		}
		arduinoState[key] = value;
	});
	if(stateChangeFound){
		broadcastData(JSON.stringify({state:arduinoState}))
	}
}

// maps file extention to MIME types
const mimeType = {
	'.ico': 'image/x-icon',
	'.html': 'text/html',
	'.js': 'text/javascript',
	'.json': 'application/json',
	'.css': 'text/css',
	'.png': 'image/png',
	'.jpg': 'image/jpeg',
	'.wav': 'audio/wav',
	'.mp3': 'audio/mpeg',
	'.svg': 'image/svg+xml',
	'.pdf': 'application/pdf',
	'.doc': 'application/msword',
	'.eot': 'appliaction/vnd.ms-fontobject',
	'.ttf': 'aplication/font-sfnt'
};


function initHttpServer(){

	httpServer.listen(config.httpPort).on('error',function(){
		console.error(`Fatal Error! Failed to listen on port ${config.httpPort}. Is something else using it?`);
		process.exit(1);
	});

	console.log('http listening on port '+config.httpPort);

}

const httpServer = http.createServer(function (req, res) {
	console.log(`${req.method} ${req.url}`);

	let parsedUrl = url.parse(req.url);
	//console.log(`parsedUrl.pathname = ${parsedUrl.pathname}`);

	// extract URL path
	// Avoid https://en.wikipedia.org/wiki/Directory_traversal_attack
	// e.g curl --path-as-is http://localhost:9000/../fileInDanger.txt
	// by limiting the path to current directory only
	const sanitizePath = path.normalize(parsedUrl.pathname).replace(/^(\.\.[\/\\])+/, '');
	let pathname = path.join(__dirname, 'webroot/', sanitizePath);

	fs.exists(pathname, function (exist) {
		if (!exist) {
			// if the file is not found, return 404
			console.log('404 not found!');
			res.statusCode = 404;
			res.end(`File ${pathname} not found!`);
			return;
		}

		// if is a directory, then look for index.html
		if (fs.statSync(pathname).isDirectory()) {
			pathname += '/index.html';
		}

		// read file from file system
		fs.readFile(pathname, function (err, data) {
			if (err) {
				console.log('500 file exists but cant read!');
				res.statusCode = 500;
				res.end(`Error getting the file: ${err}.`);
			} else {
				// based on the URL path, extract the file extention. e.g. .js, .doc, ...
				const ext = path.parse(pathname).ext;
				// if the file is found, set Content-type and send data
				res.setHeader('Content-type', mimeType[ext] || 'text/plain');
				res.end(data);
			}
		});
	});

});

httpServer.on('upgrade', function upgrade(request, socket, head) {
	const pathname = url.parse(request.url).pathname;

	switch(pathname){
		case '/wsapi': 
			webSocketServer.handleUpgrade(request, socket, head, function done(ws) {
				webSocketServer.emit('connection', ws, request);
			});
			break;
		default:
			socket.destroy();
	}
});

const webSocketServer = new WebSocket.Server({ noServer: true });

webSocketServer.on('connection', function connection(ws, req) {

	const ip = req.connection.remoteAddress;
	console.log('connection from ' + ip);

	 ws.on('message', function(message){
		console.log('message recieved',message);
		handleWebsocketMessage(ws, message);
	});

	// let interval = setInterval(function(){
	//  ws.send(JSON.stringify({state:arduinoState}));
	// }, 1000);

	ws.on('close', function clear() {
		console.log('connection closed for ' + ip);
		//clearInterval(interval);
	});

	//ws.send(JSON.stringify({connected:true}));
	ws.send(JSON.stringify({state:arduinoState}));
});

function broadcastData(data) {
	webSocketServer.clients.forEach(clientSocket => {
		if (clientSocket.readyState === WebSocket.OPEN) {
			clientSocket.send(data);
		}
	});
}

function handleWebsocketMessage(wsp, message) {
	var ws = wsp || false;
	//console.log('control, recv: "%s"', message);
	
	let messageJson;
	try{
		messageJson = JSON.parse(message);
	}catch(err) {
		console.log('control,invalid json message');
		return;
	}
	if(!messageJson.action){
		console.log('control, invalid json message, missing action');
		return;
	}
	switch(messageJson.action){
		default:
			console.log("control, unknown action!", messageJson.action);
			return;
		case "cmd":
			if(!messageJson.hasOwnProperty("value")){
				console.log('cmd, missing value');
				return;
			}
			if(!messageJson.hasOwnProperty('state')){
				console.log('cmd, missing state');
				return;
			}
			if(!messageJson.hasOwnProperty('stateValue')){
				console.log('cmd, missing state value');
				return;
			}
			if(!arduinoCommands.hasOwnProperty(messageJson.value)){
				console.log('cmd, invalid cmd');
				return;
			}

			arduinoState[messageJson.state] = messageJson.stateValue;

			broadcastData(JSON.stringify({state:arduinoState}))

			console.log('sending arduino command '+messageJson.value);
			console.log(arduinoCommands[messageJson.value]);

			arduinoSerial.write(arduinoCommands[messageJson.value]+'\n', function(err) {
				if (err) {
					return console.log('arduinoSerial Error on write: ', err.message)
				}
				console.log('arduinoSerial message written')
			})

			break;
	}
}

// function getRandomInt(max) {
//   return Math.floor(Math.random() * Math.floor(max));
// }
//
// function intervalFunc() {
//  let rand = getRandomInt(9)+'';
//   arduinoSerial.write(rand);
//   console.log(rand);
// }

console.log('init');
init();

process.on("SIGINT", function () {
	console.log("\nGracefully shutting down from SIGINT (Ctrl-C)");

	process.exit(-1);
});