# ROS2_autoBuilder.cmake [English version here](https://github.com/Doraemonjayo/ROS2_autoBuilder_cmake/blob/main/README.md)

`ROS2_autoBuilder.cmake`は、ソースディレクトリの構造に基づいてROS2プロジェクトのビルドを自動化するためのCMakeファイルです。このCMakeファイルを使用することで、ノードのビルドやメッセージ、サービス、アクションインターフェースの生成を簡単に扱うことができます。

## 機能
- **自動ノードビルド**: `nodes`ディレクトリ内のすべてのノードソースファイル（`.c`、`.cpp`など）を自動的にビルドします。
- **ライブラリ統合**: `src`ディレクトリで見つかったすべてのライブラリソースを各ノードにリンクします。
- **メッセージ、サービス、アクションのインターフェース生成**: `msg`、`srv`、`action`ディレクトリ内のファイルから自動的にインターフェースを生成します。
- **自動依存関係検出**: 必要なパッケージを自動的に検索し、依存関係を設定します。

## 追加機能
- **カスタムインターフェースの強制的な含有**: `ROS2_useCustomInterfaces_Force`関数を使用すると、カスタムメッセージを外部パッケージから含めることができます。この機能は、外部パッケージからカスタムメッセージを含むことができないROS2 Humbleのバグに対処するために追加されました。

## 使い方

1. このCMakeファイルを`CMakeLists.txt`に含めます。

   ```cmake
   include(ROS2_autoBuilder.cmake)
   ```

2. ノードをビルドするには、`ROS2_autoBuildNodes()`関数を使用し、必要なパッケージを引数として渡します。

   ```cmake
   ROS2_autoBuildNodes(<package1> <package2> ...)
   ```

3. メッセージ、サービス、アクションインターフェースを生成するには、`ROS2_autoGenerateInterfaces()`関数を使用し、必要なパッケージを引数として渡します。

   ```cmake
   ROS2_autoGenerateInterfaces(<package1> <package2> ...)
   ```

4. カスタムインターフェースを強制的に含めるには、`ROS2_useCustomInterfaces_Force()`関数を使用し、必要なパッケージを引数として渡します。

   ```cmake
   ROS2_useCustomInterfaces_Force(<package1> <package2> ...)
   ```

### `ROS2_useCustomInterfaces_Force`関数の説明
`ROS2_useCustomInterfaces_Force`関数は、指定した依存パッケージからカスタムインターフェースを含めるために使用されます。この関数は、各依存パッケージのインクルードディレクトリを設定し、対応するソースファイルを検索し、それらをノードにリンクします。この関数は、外部パッケージからカスタムメッセージを含むことができないというROS2 Humbleのバグに対処するために設計されています。指定された依存パッケージに含まれるすべてのインターフェースソースファイルを収集し、それらをすべてのノードにリンクします。

## 特定のディレクトリ構造とファイル例
このCMakeファイルは、以下のディレクトリ構造とファイルレイアウトで動作します。フォルダ内にファイルが存在しない場合、そのフォルダは省略できます。

```
.
├── CMakeLists.txt                      # プロジェクトのCMake設定ファイル
├── ROS2_autoBuilder.cmake              # このプロジェクトのCMake自動ビルダー設定
├── nodes/                              # ノードのソースファイル用ディレクトリ
│   ├── node1.cpp                       # ノード1（メイン関数を含む）
│   └── node2.cpp                       # ノード2（メイン関数を含む）
├── src/                                # すべてのノードで共有されるライブラリソース用ディレクトリ
│   ├── common_functions.cpp            # ライブラリソースファイル（ノード間で共通）
│   └── helper.cpp                      # ヘルパー関数用ファイル
├── include/                            # ヘッダファイル用ディレクトリ
│   ├── common_functions.hpp            # ライブラリのヘッダファイル
│   └── helper.hpp                      # ヘルパー関数用ヘッダファイル
├── msg/                                # メッセージ定義ファイル用ディレクトリ
│   └── ExampleMessage.msg              # メッセージ定義ファイル
├── srv/                                # サービス定義ファイル用ディレクトリ
│   └── ExampleService.srv              # サービス定義ファイル
└── action/                             # アクション定義ファイル用ディレクトリ
    └── ExampleAction.action            # アクション定義ファイル
```

### `nodes/`
`nodes`ディレクトリには**実行可能ノードのソースファイル**が含まれています。これらのファイルには`main`関数が含まれ、**独立したROS2ノード**として機能します。

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
`src`ディレクトリには、ノード間で共有されるライブラリのソースが含まれています。このディレクトリ内のソースファイルは、ビルドされる**すべてのノード**にリンクされます。

- 例: `common_functions.cpp`
  ```cpp
  #include "common_functions.hpp"

  void example_function() {
      // 共通処理
  }
  ```

- 例: `helper.cpp`
  ```cpp
  #include "helper.hpp"

  void helper_function() {
      // ヘルパー関数の実装
  }
  ```

### `include/`
`include`ディレクトリには、ライブラリソースとノードによってインクルードされるヘッダファイルが含まれています。

- 例: `common_functions.hpp`
  ```cpp
  void example_function();
  ```

- 例: `helper.hpp`
  ```cpp
  void helper_function();
  ```

### `msg/`, `srv/`, `action/`
これらのディレクトリは、ROS2のメッセージ、サービス、アクション定義ファイルを格納するためのものです。これらのフォルダにファイルが存在しない場合は、省略できます。

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

## CMakeLists.txtの例
以下は、`CMakeLists.txt`の具体的な例です。このファイルは、必要な依存パッケージ名を変更するだけで、異なるROS2パッケージで使用できます。プロジェクト名は、ディレクトリ名に基づいて自動的に設定されます。

```cmake
cmake_minimum_required(VERSION 3.16.3)

# ソースディレクトリ名からプロジェクト名を取得
get_filename_component(PROJECT_NAME_FROM_FOLDER ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT_NAME_FROM_FOLDER})

include(ROS2_autoBuilder.cmake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)

# ビルドに必要なノード依存関係を指定
ROS2_autoBuildNodes(rclcpp std_msgs)

# 生成するメッセージインターフェース依存関係を指定
ROS2_autoGenerateInterfaces(std_msgs)

ament_package()
```

この`CMakeLists.txt`は、必要なパッケージ依存関係を変更することで、異なるパッケージで使用できます。

## 注意
- `nodes/`には、`main`関数を含む実行ファイルを置いてください。
- `src/`ディレクトリには、すべてのノードでリンクされる共通ライブラリを置いてください。
- `msg/`、`srv/`、`action/`ディレクトリにファイルが存在しない場合、これらのフォルダは省略できます。

## ライセンス
このプロジェクトは[MITライセンス](LICENSE)の下でライセンスされています。
