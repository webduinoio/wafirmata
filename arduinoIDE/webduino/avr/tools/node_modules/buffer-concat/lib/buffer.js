/*!
 * buffer-concat - lib/buffer.js
 * Copyright(c) 2012 fengmk2 <fengmk2@gmail.com>
 * MIT Licensed
 *
 * Copy from: https://github.com/joyent/node/blob/master/lib/buffer.js#L504
 */

"use strict";

function bufferConcat (list, length) {
  if (!Array.isArray(list)) {
    throw new Error('Usage: bufferConcat(list, [length])');
  }

  if (list.length === 0) {
    return new Buffer(0);
  } else if (list.length === 1) {
    return list[0];
  }

  if (typeof length !== 'number') {
    length = 0;
    for (var i = 0; i < list.length; i++) {
      var buf = list[i];
      length += buf.length;
    }
  }

  var buffer = new Buffer(length);
  var pos = 0;
  for (var i = 0; i < list.length; i++) {
    var buf = list[i];
    buf.copy(buffer, pos);
    pos += buf.length;
  }
  return buffer;
}

module.exports = bufferConcat
