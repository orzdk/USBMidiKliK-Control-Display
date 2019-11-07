const { isText, isBinary, getEncoding } = require('istextorbinary');
var fs = require("fs");

var serialport = require("serialport");
const Delimiter = require('@serialport/parser-delimiter')
const Ready = require('@serialport/parser-ready')

var portName = '/dev/ttyAMA0'; 
var readData = ''; 

var displayIDX = -1;
var sysexFileList = [];

var sysexBufferString;
var sysexBuffer;
var sysexBufferPacket;

const sysexFolder = "./sysex";

var headerMessage = Buffer.from("FD", "hex");
var headerSysex = Buffer.from("FE", "hex");
var headerEnd = Buffer.from("FF", "hex");

String.prototype.replaceAll = function(search, replace) {
    var that = this;
    return that.replace(new RegExp(search, 'g'), replace);

}

function makeid(length) {
   var result           = '';
   var characters       = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789';
   var charactersLength = characters.length;
   for ( var i = 0; i < length; i++ ) {
      result += characters.charAt(Math.floor(Math.random() * charactersLength));
   }
   return result;
}

var sp = new serialport(portName, {
  baudRate: 9600,
  dataBits: 8,
  parity: 'none',
  stopBits: 1,
  flowControl: false

});

const parser = sp.pipe(new Ready({ delimiter: '÷' }))

parser.on('data', function(data){

	if (recordMode == true){
		
		newFileName = makeid(5) + ".txt";	

		fs.writeFile(newFileName, data, function (err) {
		  if (err) console.log(err);
		  console.log('Saved!');
		});

	}

});

var recordMode = false;
var ssPackage;

sp.on('data', function (data) {

	var command = data.toString();
	console.log("command",command);

	if (recordMode == true) return;

	if (command == 0x1) { //Goto next file 
		
		console.log("Received NEXT");

		if (++displayIDX > sysexFileList.length-1){ 
			displayIDX = 0;
		}	

		bufStrBuf = Buffer.from(sysexFileList[displayIDX], "utf8");
		ssPackage = Buffer.concat([ headerMessage, bufStrBuf, headerEnd ]);
		sp.write(ssPackage);	


	}
	
	else if (command == 0x2) { //Goto prev file 
	
		console.log("Received PREV");

		if (--displayIDX < 0 ){ 
			displayIdx = sysexFileList.length-1;

		}	

		bufStrBuf = Buffer.from(sysexFileList[displayIDX], "utf8");
		ssPackage = Buffer.concat([ headerMessage, bufStrBuf, headerEnd ]);
		sp.write(ssPackage);	

	}

	else if (command == 0x3) { //Send file contents

		console.log("Received SEND");

		var filepath = sysexFolder + "/" + sysexFileList[displayIDX];

		fs.readFile(filepath, function(err, buf) {

			if (buf){
				bufStrBuf = Buffer.from(buf.toString().replaceAll(" ",""), "hex");	
				ssPackage = Buffer.concat([ headerSysex, bufStrBuf, headerEnd ]);
				sp.write(ssPackage);
	 		}
		});
	}

	else if (command == 0x4) { //Record mode
		console.log("Received REC");
		recordMode = true;
		bufStrBuf = Buffer.from("[Ready for sysex]", "utf8");
		sysexBufferPacket = Buffer.concat([ headerMessage, bufStrBuf, headerEnd ]);
		sp.write(sysexBufferPacket);
	}

	else if (command == 0x5) { //Just send currently selected filename
		bufStrBuf = Buffer.from(sysexFileList[displayIDX], "utf8");
		ssPackage = Buffer.concat([ headerMessage, bufStrBuf, headerEnd ]);
		sp.write(ssPackage);	
	}

});


sp.on('close', function (err) {
	console.log('port closed');
});

sp.on('error', function (err) {
	console.error("error", err);
});

sp.on('open', function () {
	console.log('port opened...');
});

getFolderContents = function(){
	fs.readdir(sysexFolder, (err, files) => {
	  files.forEach(file => {
	    sysexFileList.push(file);
	  });
	});

}

getFolderContents();






