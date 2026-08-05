package vive

import (
	"fmt"
	"os"
	"path/filepath"
)

// findBundledLibsurvive locates the bundled libsurvive tree relative to the
// module executable. The binary lives in bin/ both in the extracted module
// package and in a dev checkout, with libsurvive/ next to bin/ at the root
// (the same layout run.sh assumes for LD_LIBRARY_PATH), so the parent of the
// executable's directory is checked first.
func findBundledLibsurvive(exePath string) (string, error) {
	exeDir := filepath.Dir(exePath)
	candidates := []string{
		filepath.Join(filepath.Dir(exeDir), "libsurvive"),
		filepath.Join(exeDir, "libsurvive"),
	}
	for _, dir := range candidates {
		if fi, err := os.Stat(dir); err == nil && fi.IsDir() {
			return dir, nil
		}
	}
	return "", fmt.Errorf("bundled libsurvive not found near %s (checked %v)", exePath, candidates)
}

// bundledLibsurvive is findBundledLibsurvive anchored at the current executable.
func bundledLibsurvive() (string, error) {
	exePath, err := os.Executable()
	if err != nil {
		return "", fmt.Errorf("cannot determine executable path: %w", err)
	}
	return findBundledLibsurvive(exePath)
}

// pluginLibPath returns the shared library path inside a libsurvive tree,
// preferring the Linux .so and falling back to the macOS .dylib.
func pluginLibPath(libsurviveDir string) string {
	so := filepath.Join(libsurviveDir, "lib", "libsurvive.so")
	if _, err := os.Stat(so); err == nil {
		return so
	}
	return filepath.Join(libsurviveDir, "lib", "libsurvive.dylib")
}
