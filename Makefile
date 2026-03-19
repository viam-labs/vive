MODULE_BINARY := bin/vive
CLI_BINARY := bin/vive-cli

LIBSURVIVE_REPO ?= https://github.com/viam-labs/libsurvive.git
LIBSURVIVE_REF  ?= viam-patches
LIBSURVIVE_DIR  := libsurvive
LIBSURVIVE_SRC  := libsurvive-src
LIBSURVIVE_LIB  := $(LIBSURVIVE_DIR)/lib/libsurvive.so

# On macOS, help cmake find homebrew packages and use HIDAPI backend
CMAKE_EXTRA :=
ifneq ($(wildcard /opt/homebrew),)
  CMAKE_EXTRA += -DCMAKE_PREFIX_PATH=/opt/homebrew -DLIBUSB_LIBRARY=/opt/homebrew/lib/libusb-1.0.dylib -DUSE_HIDAPI=ON
  LIBSURVIVE_LIB := $(LIBSURVIVE_DIR)/lib/libsurvive.dylib
endif

-include .env
export

HZ ?= 90
POS_DEADZONE ?= 0.5
ROT_DEADZONE ?= 1.0
SMOOTH_ALPHA ?= 0.5

# Build libsurvive from source (first time only)
$(LIBSURVIVE_LIB):
	@[ -d $(LIBSURVIVE_SRC) ] || git clone --depth 1 --recurse-submodules --branch $(LIBSURVIVE_REF) $(LIBSURVIVE_REPO) $(LIBSURVIVE_SRC)
	cd $(LIBSURVIVE_SRC) && mkdir -p build && cd build && \
		cmake .. -DCMAKE_INSTALL_PREFIX=$(CURDIR)/$(LIBSURVIVE_DIR) -DCMAKE_BUILD_TYPE=Release $(CMAKE_EXTRA) && \
		$(MAKE) -j$$(nproc 2>/dev/null || sysctl -n hw.ncpu) && $(MAKE) install
	@# macOS: libsurvive plugin loader searches for .so but cmake installs .dylib
	@for f in $(LIBSURVIVE_DIR)/lib/libsurvive/plugins/*.dylib; do \
		[ -f "$$f" ] && ln -sf "$$(basename $$f)" "$${f%.dylib}.so"; \
	done 2>/dev/null; true

# Module binary (for Viam module deployment)
$(MODULE_BINARY): $(LIBSURVIVE_LIB) Makefile go.mod *.go survive/*.go cmd/module/*.go
	go build -o $(MODULE_BINARY) ./cmd/module

# CLI binary (for remote development/testing)
$(CLI_BINARY): $(LIBSURVIVE_LIB) Makefile go.mod *.go survive/*.go cmd/cli/*.go
	go build -o $(CLI_BINARY) ./cmd/cli

build: $(MODULE_BINARY) $(CLI_BINARY)

# Run CLI with env vars from .env
dev: $(CLI_BINARY)
	GOTRACEBACK=crash ./$(CLI_BINARY) \
		--address $(VIAM_ADDRESS) --key-id $(VIAM_KEY_ID) --key $(VIAM_KEY) \
		--hz $(HZ) \
		--left-arm $(LEFT_ARM) --left-gripper $(LEFT_GRIPPER) \
		--right-arm $(RIGHT_ARM) --right-gripper $(RIGHT_GRIPPER) \
		--pos-deadzone $(POS_DEADZONE) --rot-deadzone $(ROT_DEADZONE) \
		--smooth-alpha $(SMOOTH_ALPHA)

recalibrate:
	@if [ -f ~/.config/libsurvive/config.json ]; then \
		cp ~/.config/libsurvive/config.json ~/.config/libsurvive/config.json.bak; \
		rm ~/.config/libsurvive/config.json; \
		echo "Cleared libsurvive config (backup: config.json.bak)"; \
	fi
	@rm -f calibration.json
	@echo "Cleared calibration.json — recalibrate forward direction with trackpad-up gesture"
	$(MAKE) dev

pair: $(LIBSURVIVE_LIB)
	@echo 'Plug in both Watchman dongles, then power on each controller one at a time.'
	@echo 'Press Ctrl-C when both controllers are paired.'
	DYLD_LIBRARY_PATH=$(CURDIR)/$(LIBSURVIVE_DIR)/lib LD_LIBRARY_PATH=$(CURDIR)/$(LIBSURVIVE_DIR)/lib \
		$(LIBSURVIVE_DIR)/bin/survive-cli --pair-device

module.tar.gz: meta.json $(MODULE_BINARY)
	tar czf $@ meta.json $(MODULE_BINARY) $(LIBSURVIVE_DIR)/lib

module: test module.tar.gz

test:
	go test ./...

setup:
ifeq ($(shell uname -s),Darwin)
	brew install cmake libusb hidapi zlib
else
	sudo apt-get install -y cmake libusb-1.0-0-dev zlib1g-dev
endif
	go mod tidy

update:
	go get go.viam.com/rdk@latest
	go mod tidy

lint:
	gofmt -s -w .

clean:
	rm -rf $(LIBSURVIVE_DIR) $(LIBSURVIVE_SRC) bin/ module.tar.gz

.PHONY: build dev recalibrate pair module test setup update lint clean
