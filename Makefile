UWB_ENV = LOCAL_USER_ID=$$(id -u) LOCAL_GROUP_ID=$$(id -g) LOCAL_GROUP_NAME=$$(id -gn)

uwb.up:
	@xhost +local:docker
	@docker rm -f uwb >/dev/null 2>&1 || true
	@$(UWB_ENV) docker compose up -d --build
uwb.down:
	@xhost +local:docker
	@docker compose down
	@docker rm -f uwb >/dev/null 2>&1 || true
uwb.restart:
	@xhost +local:docker
	@docker compose restart
uwb.shell:
	@xhost +local:docker
	@docker exec --user "$$(id -u):$$(id -g)" -it uwb bash
uwb.build:
	@$(UWB_ENV) docker compose build
	