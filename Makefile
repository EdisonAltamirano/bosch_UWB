# Platform detection: Windows_NT (GNU Make on Windows), Darwin (macOS), else Linux
UNAME_S := $(shell uname -s 2>/dev/null)

ifeq ($(OS),Windows_NT)
    PLATFORM := windows
else ifeq ($(UNAME_S),Darwin)
    PLATFORM := macos
else
    PLATFORM := linux
endif

ifeq ($(PLATFORM),windows)
    COMPOSE_FILES := -f compose.yaml -f compose.windows.yaml
else ifeq ($(PLATFORM),macos)
    COMPOSE_FILES := -f compose.yaml -f compose.mac.yaml
else
    COMPOSE_FILES := -f compose.yaml -f compose.linux.yaml
endif

COMPOSE := docker compose $(COMPOSE_FILES)

# Linux-only: allow Docker containers to use the host X11 socket
ifeq ($(PLATFORM),linux)
    XHOST := xhost +local:docker
else
    XHOST :=
endif

# Map container UID/GID to the host user (not needed on Windows)
ifeq ($(PLATFORM),windows)
    USER_ENV :=
    DOCKER_EXEC_USER :=
else
    USER_ENV := LOCAL_USER_ID=$$(id -u) LOCAL_GROUP_ID=$$(id -g) LOCAL_GROUP_NAME=$$(id -gn)
    DOCKER_EXEC_USER := --user "$$(id -u):$$(id -g)"
endif

# macOS + XQuartz: allow Docker GUI (live RDM viewer, gedit, etc.)
ifeq ($(PLATFORM),macos)
    XHOST_MAC := xhost +localhost
else
    XHOST_MAC :=
endif

.PHONY: uwb.up uwb.down uwb.restart uwb.shell uwb.build uwb.xhost

uwb.up:
	@$(XHOST)
	@$(USER_ENV) $(COMPOSE) up -d

uwb.down:
	@$(XHOST)
	@$(COMPOSE) stop

uwb.restart:
	@$(XHOST)
	@$(COMPOSE) restart

uwb.shell:
	@$(XHOST)
	@docker exec $(DOCKER_EXEC_USER) -it uwb bash

uwb.build:
	@$(USER_ENV) $(COMPOSE) build

uwb.xhost:
	@$(XHOST_MAC)
