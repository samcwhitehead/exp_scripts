#!/usr/bin/env python

import os
import warnings

def get_SHA_keys(repo_dirs):
	repo_dirs = [l.rstrip() for l in repo_dirs]
	git_SHA = ''.join([p + ':'+ os.popen('git -C %s rev-parse HEAD'%(p)).read() for p in repo_dirs])
	return git_SHA

def check_git_status(repo_dirs):
	import numpy as np
	repo_dirs = [l.rstrip() for l in repo_dirs]
	status_strings = [os.popen('git -C %s status'%(p)).read() for p in repo_dirs]
	status = np.array([not("Changes not staged for commit:" in s) for s in status_strings])
	if np.sum(~status):
		print 'the following repositories contain uncommited edits in tracked files:'
		print [repo_dirs[i] for i,s in enumerate(status) if not(s)]
		return False
	else:	
		print 'all tracked repositories are up-to-date'
		return True