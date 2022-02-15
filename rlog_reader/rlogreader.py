#!/usr/bin/python
import os
import sys
import gzip
import zlib
import json
import bz2
import tempfile
import requests
import subprocess32 as subprocess
from aenum import Enum
import matplotlib.pyplot as plt
import numpy as np
import scipy
import platform
if platform.system() == "Darwin":
  os.environ["LA_LIBRARY_FILEPATH"] = "/usr/local/opt/libarchive/lib/libarchive.dylib"
import libarchive.public

from cereal import log as capnp_log

OP_PATH = os.path.dirname(os.path.dirname(capnp_log.__file__))
class DataUnreadableError(Exception):
  pass
def FileReader(fn):
  return open(fn, 'rb')

# this is an iterator itself, and uses private variables from LogReader
class MultiLogIterator(object):
  def __init__(self, log_paths, wraparound=True):
    self._log_paths = log_paths
    self._wraparound = wraparound

    self._first_log_idx = next(i for i in xrange(len(log_paths)) if log_paths[i] is not None)
    self._current_log = self._first_log_idx
    self._idx = 0
    self._log_readers = [None]*len(log_paths)
    self.start_time = self._log_reader(self._first_log_idx)._ts[0]

  def _log_reader(self, i):
    if self._log_readers[i] is None and self._log_paths[i] is not None:
      log_path = self._log_paths[i]
      print("LogReader:" + log_path)
      self._log_readers[i] = LogReader(log_path)

    return self._log_readers[i]

  def __iter__(self):
    return self

  def _inc(self):
    lr = self._log_reader(self._current_log)
    if self._idx < len(lr._ents)-1:
      self._idx += 1
    else:
      self._idx = 0
      self._current_log = next(i for i in xrange(self._current_log + 1, len(self._log_readers) + 1) if i == len(self._log_readers) or self._log_paths[i] is not None)
      # wraparound
      if self._current_log == len(self._log_readers):
        if self._wraparound:
          self._current_log = self._first_log_idx
        else:
          raise StopIteration

  def next(self):
    while 1:
      lr = self._log_reader(self._current_log)
      ret = lr._ents[self._idx]
      if lr._do_conversion:
        ret = convert_old_pkt_to_new(ret, lr.data_version)
      self._inc()
      return ret

  def tell(self):
    # returns seconds from start of log
    return (self._log_reader(self._current_log)._ts[self._idx] - self.start_time) * 1e-9

  def seek(self, ts):
    # seek to nearest minute
    minute = int(ts/60)
    if minute >= len(self._log_paths) or self._log_paths[minute] is None:
      return False

    self._current_log = minute

    # HACK: O(n) seek afterward
    self._idx = 0
    while self.tell() < ts:
      self._inc()
    return True

class LogReader:
  def __init__(self, fn, canonicalize=True, only_union_types=False, sort_by_time=False):
    data_version = None
    _, ext = os.path.splitext(fn)
    with FileReader(fn) as f:
      dat = f.read()

    if ext == "":
      # old rlogs weren't bz2 compressed
      ents = capnp_log.Event.read_multiple_bytes(dat)
    elif ext == ".bz2":
      dat = bz2.decompress(dat)
      ents = capnp_log.Event.read_multiple_bytes(dat)
    else:
      raise Exception("unknown extension " + ext)

    self._ents = list(sorted(ents, key=lambda x: x.logMonoTime) if sort_by_time else ents)
    self._ts = [x.logMonoTime for x in self._ents]
    self.data_version = data_version
    self._only_union_types = only_union_types

  def __iter__(self):
    for ent in self._ents:
      if self._only_union_types:
        try:
          ent.which()
          yield ent
        except capnp.lib.capnp.KjException:
          pass
      else:
        yield ent

def load_many_logs_canonical(log_paths):
  """Load all logs for a sequence of log paths."""
  for log_path in log_paths:
    for msg in LogReader(log_path):
      yield msg

if __name__ == "__main__":
  steering_angles = [] # Empty list 
  log_path = sys.argv[1]
  lr = LogReader(log_path)
  logs = list(lr)
  # The following shows you which
  for i in logs:
	  #print(i.which())
    if i.which() == "sensorEvents":
      print (i.sensorEvents)
  #print([l.carControl.actuators.steerAngle for l in logs if l.which == "carControl"]);
  # for l in logs:
  #   if l.which == "carControl":
  #     print(l.carControl.actuators.steerAngle)

