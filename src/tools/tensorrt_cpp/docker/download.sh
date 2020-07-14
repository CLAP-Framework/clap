#!/bin/bash

function usage() {
    echo "Usage: $0 [OPTIONS], copt the docker file to the NVIDIA platfrom."
    echo "    -h,--help                Display the usage and exit."
    echo "    -c, --cp                 The file will be copied to the NVIDIA platfrom."
}

function usagescp() {
    echo "    example: "
    echo "    ./download.sh -c src_file_path ip_add dest_user_name"
}

while true; do
        case $1 in
        -h|--help)
                usage
                exit 0
            ;;
                -c|--cp)
                    if [ $# -lt 4 ];
                        then
                                usagescp
                                exit 0
                        fi
                        src_file=$2
                        ip_add=$3
                        user_name=$4
                        command=$user_name"@"$ip_add":/home/"$user_name"/workspace/cross_compiler_dir"
                        echo $command
                        scp -r $src_file $user_name"@"$ip_add":/home/"$user_name"/workspace/cross_compiler_dir"
                        exit 0
                        ;;
                *)
                        usage
                exit 0
            ;;
  esac
done