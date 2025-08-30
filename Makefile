run_docker:
	xhost +local:docker && chmod -R a+rw . && docker compose -f compose.yml up
exec_container:
	docker compose -f compose.yml exec ros2_eval_task /bin/bash