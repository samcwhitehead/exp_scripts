#!/usr/bin/env python

import os
import warnings

def get_SHA_keys(repo_dirs):
	repo_dirs = [l.rstrip() for l in repo_dirs]
	git_SHA = ''.join([p + ':'+ os.popen('git -C %s rev-parse HEAD'%(p)).read() for p in repo_dirs])
	return git_SHA

def check_git_status(repo_dirs):
	repo_dirs = [l.rstrip() for l in repo_dirs]
	status_strings = [p + ':'+ os.popen('git -C %s status'%(p)).read() for p in repo_dirs]
	for s in status_strings:
		print s
	status = [("""Your branch is up-to-date with 'origin/master'.""" in s) for s in status_strings]
	import numpy as np
	if np.sum(status):
		print 'all tracked repositories are up-to-date'
		return True
	else:
		print 'the following repositories contain uncommited edits in tracked files:'
		print [repo_dirs[i] for i,s in enumerate(status) if not(s)]
		return False