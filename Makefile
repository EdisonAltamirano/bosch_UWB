# Detect platform — GNU Make sets OS=Windows_NT on Windows
ifeq ($(OS),Windows_NT)
    COMPOSE_FILES := -f compose.yaml -f compose.windows.yaml
else
    COMPOSE_FILES := -f compose.yaml -f compose.linux.yaml
endif

COMPOSE := docker compose $(COMPOSE_FILES)

.PHONY: uwb.up uwb.down uwb.restart uwb.shell uwb.build

uwb.up:
ifeq ($(OS),Windows_NT)
	@$(COMPOSE) up -d
else
	@xhost +local:docker
	@LOCAL_USER_ID=$$(id -u) LOCAL_GROUP_ID=$$(id -g) LOCAL_GROUP_NAME=$$(id -gn) $(COMPOSE) up -d
endif

uwb.down:
ifeq ($(OS),Windows_NT)
	@$(COMPOSE) stop
else
	@xhost +local:docker
	@$(COMPOSE) stop
endif

uwb.restart:
ifeq ($(OS),Windows_NT)
	@$(COMPOSE) restart
else
	@xhost +local:docker
	@$(COMPOSE) restart
endif

uwb.shell:
ifeq ($(OS),Windows_NT)
	@docker exec -it uwb bash
else
	@xhost +local:docker
	@docker exec --user "$$(id -u):$$(id -g)" -it uwb bash
endif

uwb.build:
ifeq ($(OS),Windows_NT)
	@$(COMPOSE) build
else
	@LOCAL_USER_ID=$$(id -u) LOCAL_GROUP_ID=$$(id -g) LOCAL_GROUP_NAME=$$(id -gn) $(COMPOSE) build
endif
