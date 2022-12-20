#!/bin/bash

PREFIX="origin/dev-6.0"
LOCAL_PREFIX=$(echo ${PREFIX}|sed "s|origin/||")

if [ -d ".git/sequencer" ]; then
  echo "FATAL ERROR: Cherry-pick or rebase in progress. ABORTING."
  exit 1
fi

git fetch
git checkout ${PREFIX}_base
git branch -D ${PREFIX}

BRANCH_CMD="git branch --list"
if [[ ${PREFIX} == "origin/"* ]]; then
	BRANCH_CMD+=" -r"
fi

BRANCHES_TO_MERGE=""
BRANCHES=$(${BRANCH_CMD} |grep $PREFIX|xargs)
echo "Branches to merge:"
for b in ${BRANCHES}; do
  if [[ "${b}" == "${PREFIX}"* ]]; then
    if [ "${b}" == "${PREFIX}" ]; then continue; fi
    if [ "${b}" == "${PREFIX}_base" ]; then continue; fi
    if [[ "${b}" =~ "${PREFIX}-"[0-9]* ]]; then continue; fi
    BRANCHES_TO_MERGE="${BRANCHES_TO_MERGE} ${b}"
    echo "${b}"
  fi
done
echo ""
git merge --no-edit ${BRANCHES_TO_MERGE}
git branch -D ${LOCAL_PREFIX}
git branch ${LOCAL_PREFIX}
git checkout ${LOCAL_PREFIX}
echo "**************************************"
echo "Done merging! Now push it\n"
echo "**************************************"

