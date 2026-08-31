{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";  # IMPORTANT!!!

    nixpkgs-unstable.url = "github:nixos/nixpkgs/nixos-unstable";
  };
  outputs = { self, nix-ros-overlay, nixpkgs, nixpkgs-unstable }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };

	unstable = import nixpkgs-unstable { inherit system; };
      in {
        devShells.default = pkgs.mkShell {
          name = "guppy_ros";
          packages = [
            # non ros
            
            # build
            pkgs.colcon
            pkgs.cmake
            pkgs.clang-tools
            
            # deps
            pkgs.proxsuite

            (unstable.python3.withPackages (ps: with ps; [
              pygame
              pip
              pyside6
            ]))

            # extra
            pkgs.fastfetch
            pkgs.can-utils
            
            (with pkgs.rosPackages.jazzy; buildEnv {
              # ros packages
              paths = [
                # ros base
                ros-core
                ros-base
                rclcpp
                rclpy
                
                # ros msgs
                std-msgs
                geometry-msgs
                sensor-msgs
                nav-msgs
      
                # rqt
                rqt
                rqt-common-plugins
                
                # build
                ament-cmake
                ament-cmake-core # vectornav_msgs
                ament-cmake-python
                ament-lint-auto
                
                # launch
                launch
                launch-ros
                launch-xml
                
                # rviz
                rviz2
                
                # deps
                ros2-control
                control-toolbox
                ros-gz-interfaces # gncea_autonomy
                python-cmake-module # vecornav_msgs
                marine-acoustic-msgs # waterlinked_dvl_driver
                ros-gz-sim # gncea_description
                ros-gz-bridge # sim
                joint-state-publisher # sim
              ];
            })
          ];
          # shellHook = ''
          #   fastfetch -l ./.github/guppy.txt
          # '';
        };
      });
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" "https://palouse-robosub.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" "palouse-robosub.cachix.org-1:r2KNmfNGOZB+IhqEqDIMDaEWMYZv8ct1tdSg7n7fNKw=" ];
  };
}
