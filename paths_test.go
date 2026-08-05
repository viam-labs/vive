package vive

import (
	"os"
	"path/filepath"
	"testing"
)

// makeLayout creates dirs under a temp root and returns the root.
func makeLayout(t *testing.T, dirs ...string) string {
	t.Helper()
	root := t.TempDir()
	for _, d := range dirs {
		if err := os.MkdirAll(filepath.Join(root, d), 0o755); err != nil {
			t.Fatal(err)
		}
	}
	return root
}

func TestFindBundledLibsurvive(t *testing.T) {
	t.Run("deployed module layout", func(t *testing.T) {
		// Extracted module package: <pkg>/bin/vive with <pkg>/libsurvive/lib.
		root := makeLayout(t, "bin", "libsurvive/lib")
		got, err := findBundledLibsurvive(filepath.Join(root, "bin", "vive"))
		if err != nil {
			t.Fatalf("unexpected error: %v", err)
		}
		if want := filepath.Join(root, "libsurvive"); got != want {
			t.Errorf("got %q, want %q", got, want)
		}
	})

	t.Run("sibling of executable", func(t *testing.T) {
		root := makeLayout(t, "libsurvive/lib")
		got, err := findBundledLibsurvive(filepath.Join(root, "vive"))
		if err != nil {
			t.Fatalf("unexpected error: %v", err)
		}
		if want := filepath.Join(root, "libsurvive"); got != want {
			t.Errorf("got %q, want %q", got, want)
		}
	})

	t.Run("missing", func(t *testing.T) {
		root := makeLayout(t, "bin")
		if _, err := findBundledLibsurvive(filepath.Join(root, "bin", "vive")); err == nil {
			t.Error("expected error when libsurvive dir does not exist")
		}
	})
}

func TestPluginLibPath(t *testing.T) {
	root := makeLayout(t, "libsurvive/lib")
	soPath := filepath.Join(root, "libsurvive", "lib", "libsurvive.so")
	if err := os.WriteFile(soPath, nil, 0o644); err != nil {
		t.Fatal(err)
	}
	if got := pluginLibPath(filepath.Join(root, "libsurvive")); got != soPath {
		t.Errorf("got %q, want %q", got, soPath)
	}

	// Without a .so present, falls back to the .dylib path (macOS build).
	dylibRoot := makeLayout(t, "libsurvive/lib")
	want := filepath.Join(dylibRoot, "libsurvive", "lib", "libsurvive.dylib")
	if got := pluginLibPath(filepath.Join(dylibRoot, "libsurvive")); got != want {
		t.Errorf("got %q, want %q", got, want)
	}
}
