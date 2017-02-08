(function () {
  "use strict";
  var bufferConcat = require('buffer-concat');
  var burnURL = process.argv.slice(2)[0];
  var infoFile = process.argv.slice(3)[0];
  var burnKey = '';
  var burnServer = 'r.webduino.io';
  var hasBurnKey = false;

  var lineReader = require('readline').createInterface({
    input: require('fs').createReadStream(infoFile)
  });

  lineReader.on('line', function (line) {
    if (line.startsWith("//Burn-Key")) {
      burnKey = line.split(' ')[1];
      hasBurnKey = true;
      start(burnKey, burnServer);
    } else if (line.startsWith("//Burn-Server")) {
      burnServer = line.split(' ')[1];
    }
  });

  setTimeout(function () {
    if (!hasBurnKey) {
      console.log("need to define //Burn-Key: ????");
      return;
    }
  }, 1000);

  function start(burnKey, burnServer) {
    console.log("Start upload...");
    console.log("Burn Key:", burnKey);
    console.log("Burn Server:", burnServer);
    var filename, path;
    if (burnURL.indexOf('/') == -1) {
      filename = burnURL;
      path = "./";
    } else {
      var endOfSlash = burnURL.lastIndexOf('/');
      path = burnURL.substring(0, endOfSlash + 1);
      filename = burnURL.substring(endOfSlash + 1);
    }

    var fs = require("fs"),
      http = require("http"),
      data = fs.readFileSync(path + filename),
      client, request;

    console.log("File:", path + filename);

    var crlf = "\r\n",
      boundary = '---------------------------10102754414578508781458777923', // Boundary: "--" + up to 70 ASCII chars + "\r\n"
      delimiter = crlf + "--" + boundary,
      headers = [
        'Content-Disposition: form-data; name="file"; filename="' + filename + '"' + crlf,
        'Content-Type: arduino/hex' + crlf,
      ],
      closeDelimiter = delimiter + "--",
      multipartBody;

    multipartBody = Buffer.concat([
      new Buffer(delimiter + crlf + headers.join('') + crlf),
      data,
      new Buffer(closeDelimiter)
    ]);
    //client = http.createClient(80, "localhost");
    //request = client.request('POST', '/upload/firmware', {
    request = http.request({
      'host': "webduino.tw",
      'port': 80,
      'path': '/upload/firmware',
      'method': 'POST',
      'headers': {
        'Burn-Key': burnKey,
        'Burn-Server': burnServer,
        'Board-Type': "fly",
        'User-Agent': 'upload',
        'Accept-Encoding': 'gzip,deflate',
        'Content-Type': 'multipart/form-data; boundary=' + boundary,
        'Content-Length': multipartBody.length
      }
    });

    request.write(multipartBody);
    request.end();

    request.on('error', function (err) {
      console.log(err);
    });

    request.on('response', function (response) {
      console.log('response');

      response.setEncoding('utf8');

      response.on('data', function (chunk) {
        console.log(chunk.toString());
      });

      response.on('end', function () {
        console.log("end");
      });
    });
  }
}());