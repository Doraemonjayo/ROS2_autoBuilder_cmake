# ROS2_autoBuilder.cmake

`ROS2_autoBuilder.cmake` は、ROS2 プロジェクトのビルドをソースディレクトリの構造に基づいて自動化するための CMake ファイルです。この CMake ファイルを使うことで、ノードのビルドやメッセージ、サービス、アクションのインターフェース生成が簡単に行えます。

## 特徴
- **ノードの自動ビルド**: `nodes` ディレクトリにある全てのノードソースファイル（`.c`, `.cpp` など）を自動でビルドします。
- **ライブラリの統合**: `src` ディレクトリにあるライブラリソースを、すべてのノードにリンクします。
- **メッセージ、サービス、アクションのインターフェース生成**: `msg`, `srv`, `action` ディレクトリ内のファイルからインターフェースを自動生成します。
- **依存関係の自動検出**: 必要なパッケージを自動で検索し、依存関係を設定します。

## 使用方法

1. `CMakeLists.txt` にこの CMake ファイルを含めます。

```cmake
include(ROS2_autoBuilder.cmake)
```

2. ノードをビルドするには、`ROS2_autoBuildNodes()` 関数を使用します。依存するパッケージを引数に渡してください。

```cmake
ROS2_autoBuildNodes(<package1> <package2> ...)
```

3. メッセージ、サービス、アクションのインターフェースを生成するには、`ROS2_autoGenerateInterfaces()` 関数を使用します。依存するパッケージを引数に渡してください。

```cmake
ROS2_autoGenerateInterfaces(<package1> <package2> ...)
```

## 具体的なディレクトリ構造とファイル例
以下のようなディレクトリ構造とファイル配置で動作します。ファイルがない場合は、フォルダは省略することが可能です。

```
.
├── CMakeLists.txt                      # プロジェクトのCMake設定ファイル
├── ROS2_autoBuilder.cmake              # このプロジェクトのCMake自動ビルド設定
├── nodes/                              # ノードのソースファイルを格納するディレクトリ
│   ├── node1.cpp                       # ノード1（main関数を含む）
│   └── node2.cpp                       # ノード2（main関数を含む）
├── src/                                # 共通のライブラリソースを格納するディレクトリ
│   ├── common_functions.cpp            # ライブラリソースファイル（ノード間で共通）
│   └── helper.cpp                      # 補助機能用のファイル
├── include/                            # ヘッダーファイルを格納するディレクトリ
│   ├── common_functions.hpp            # ライブラリ用のヘッダーファイル
│   └── helper.hpp                      # 補助機能用のヘッダーファイル
├── msg/                                # メッセージ定義ファイルを格納するディレクトリ
│   └── ExampleMessage.msg              # メッセージ定義ファイル
├── srv/                                # サービス定義ファイルを格納するディレクトリ
│   └── ExampleService.srv              # サービス定義ファイル
└── action/                             # アクション定義ファイルを格納するディレクトリ
    └── ExampleAction.action            # アクション定義ファイル
```

### `nodes/`
`nodes` ディレクトリには、**各ノードに対応する実行可能ファイル**のソースファイルを配置します。これらのファイルには `main` 関数が含まれ、それぞれが**独立した ROS2 ノード**として動作します。

- 例: `node1.cpp`
  ```cpp
  #include "rclcpp/rclcpp.hpp"
  #include "std_msgs/msg/string.hpp"

  int main(int argc, char **argv) {
      rclcpp::init(argc, argv);
      auto node = rclcpp::Node::make_shared("node1");
      RCLCPP_INFO(node->get_logger(), "Hello from Node 1!");
      rclcpp::spin(node);
      rclcpp::shutdown();
      return 0;
  }
  ```

- 例: `node2.cpp`
  ```cpp
  #include "rclcpp/rclcpp.hpp"
  #include "std_msgs/msg/string.hpp"

  int main(int argc, char **argv) {
      rclcpp::init(argc, argv);
      auto node = rclcpp::Node::make_shared("node2");
      RCLCPP_INFO(node->get_logger(), "Hello from Node 2!");
      rclcpp::spin(node);
      rclcpp::shutdown();
      return 0;
  }
  ```

### `src/`
`src` ディレクトリには、ノード間で共通して使用するライブラリソースを配置します。このディレクトリ内のソースファイルは、ビルドされた**すべてのノードにリンク**されます。

- 例: `common_functions.cpp`
  ```cpp
  #include "common_functions.hpp"

  void example_function() {
      // 共通の処理
  }
  ```

- 例: `helper.cpp`
  ```cpp
  #include "helper.hpp"

  void helper_function() {
      // 補助機能の実装
  }
  ```

### `include/`
`include` ディレクトリには、ライブラリソースやノードからインクルードされるヘッダーファイルを配置します。

- 例: `common_functions.hpp`
  ```cpp
  void example_function();
  ```

- 例: `helper.hpp`
  ```cpp
  void helper_function();
  ```

### `msg/`, `srv/`, `action/`
それぞれ、ROS2 のメッセージ、サービス、アクション定義ファイルを格納するためのディレクトリです。ファイルが存在しない場合はこれらのフォルダを省略できます。

- 例: `ExampleMessage.msg`
  ```
  string data
  ```

- 例: `ExampleService.srv`
  ```
  int32 a
  int32 b
  ---
  int32 sum
  ```

- 例: `ExampleAction.action`
  ```
  # Goal
  int32 order

  ---
  # Result
  string result

  ---
  # Feedback
  string feedback
  ```

## CMakeLists.txt の例
以下は、CMakeLists.txt の具体例です。このファイルは、依存先のパッケージ名を変更するだけで、異なる ROS2 パッケージに対応できます。プロジェクト名はディレクトリ名に基づいて自動的に設定されます。

```cmake
cmake_minimum_required(VERSION 3.16.3)

# Get the project name from the source directory name
get_filename_component(PROJECT_NAME_FROM_FOLDER ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT_NAME_FROM_FOLDER})

include(ROS2_autoBuilder.cmake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)

# ノードの依存パッケージを指定してビルド
ROS2_autoBuildNodes(rclcpp std_msgs)

# メッセージインターフェースの依存パッケージを指定して生成
ROS2_autoGenerateInterfaces(std_msgs)

ament_package()
```

この CMakeLists.txt は、依存するパッケージを変更するだけで、異なるパッケージに対応して利用できます。

## 注意事項
- `nodes/` には、各ノードの `main` 関数を含む実行可能ファイルを配置します。
- `src/` ディレクトリには、すべてのノードにリンクされる共通ライブラリを配置します。
- `msg/`, `srv/`, `action/` ディレクトリにファイルがない場合は、それらのフォルダを省略できます。

## ライセンス
このプロジェクトは [MIT License](LICENSE) の下でライセンスされています。
