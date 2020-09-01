#!/bin/bash

#place this script in /etc/bash_completion.d/ for general use.
_script()
{
        local cmd="${1##*/}"
	_script_commands=$($cmd -m $1 $2 $3)
	local cur prev
	COMPREPLY=()
	cur="${COMP_WORDS[COMP_CWORD]}"
	COMPREPLY=( $(compgen -W "${_script_commands}" -- ${cur}) )
	_filedir	
	return 0
}

complete -o nospace -F _script ./simple_pose_test
complete -o nospace -F _script ./survive-buttons
complete -o nospace -F _script ./survive-cli
complete -o nospace -F _script ./survive-websocketd
complete -o nospace -F _script simple_pose_test
complete -o nospace -F _script survive-buttons
complete -o nospace -F _script survive-cli
complete -o nospace -F _script survive-websocketd


