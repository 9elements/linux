#!/bin/bash

PREFIX="origin/dev-5.15"
LOCAL_PREFIX=$(echo ${PREFIX}|sed "s|origin/||")

if [ -d ".git/sequencer" ]; then
  echo "FATAL ERROR: Cherry-pick or rebase in progress. ABORTING."
  exit 1
fi

git fetch
git checkout ${PREFIX}_base
git branch -D ${PREFIX}

BRANCHES=$(git branch --list -r |grep $PREFIX|xargs)
for b in ${BRANCHES}; do
  if [ "${b}" == "${PREFIX}" ]; then continue; fi
  if [ "${b}" == "${PREFIX}_base" ]; then continue; fi
  if [[ "${b}" =~ "${PREFIX}-"[0-9]* ]]; then continue; fi

  echo "Commits on branch ${b}:"
  commits=$(git log --oneline ${b}...origin/dev-5.15_base --no-decorate --reverse| cut -d' ' -f1)
  echo "${commits}"
  git cherry-pick $(echo ${commits}| xargs) 
  echo ""
done
git branch -D ${LOCAL_PREFIX}
git branch ${LOCAL_PREFIX}
git checkout ${LOCAL_PREFIX}
echo "**************************************"
echo "Done merging! Now push it\n"
echo "**************************************"

