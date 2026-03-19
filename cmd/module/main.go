package main

import (
	"go.viam.com/rdk/components/input"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"

	vive "vive"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: input.API, Model: vive.ViveController},
		resource.APIModel{API: generic.API, Model: vive.ViveTeleop},
	)
}
