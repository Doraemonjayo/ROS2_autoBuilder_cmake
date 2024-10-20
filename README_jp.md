# ROS2_autoBuilder.cmake

## 概要
`ROS2_autoBuilder.cmake`は、ROS 2プロジェクトのビルド構成を自動化するために設計されたCMakeスクリプトです。このCMakeファイルは、指定されたディレクトリ構造に基づいてノードとライブラリをビルドするプロセスを簡素化し、カスタムメッセージ、サービス、アクションインターフェースの生成もサポートします。指定されたディレクトリ内のすべての`.cpp`ファイルやその他の必要なファイルを自動的に検索するため、CMakeLists.txtファイルを毎回手動で修正する必要がなくなります。

## 使用方法

### 基本的なCMakeLists.txtの例

```cmake
cmake_minimum_required(VERSION 3.16.1)

# 現在のフォルダ名からプロジェクト名を取得
get_filename_component(PROJECT_NAME_FROM_FOLDER ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT_NAME_FROM_FOLDER})

# ROS2_autoBuilder.cmakeをインクルード
include(ROS2_autoBuilder.cmake)

# C++およびCの標準を指定
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)

# カスタムインターフェース（メッセージ、サービス、アクション）を自動的に生成
ROS2_autoGenerateInterfaces(std_msgs)

# ノードを自動的にビルド
ROS2_autoBuildNodes(rclcpp std_msgs)

# amentパッケージを設定
ament_package()
```

このCMakeLists.txtファイルでは、`ROS2_autoGenerateInterfaces`と`ROS2_autoBuildNodes`関数を使用して、カスタムインターフェースの生成とノードのビルドを自動化しています。依存関係を変更するだけで、さまざまなパッケージに対応できるため、プロジェクト間の再利用性が向上します。

### ディレクトリ構造

以下のディレクトリ構造が推奨されますが、ファイルが存在しないディレクトリは省略できます。

```
your_project/
├── CMakeLists.txt
├── ROS2_autoBuilder.cmake
├── nodes/
│   ├── node1.cpp
│   └── node2.cpp
├── src/
│   ├── lib1.cpp
│   └── lib2.cpp
├── include/
│   ├── lib1.hpp
│   └── lib2.hpp
├── msg/
│   └── MyMsg.msg
├── srv/
│   └── MySrv.srv
└── action/
    └── MyAction.action
```

- **nodes/**: このディレクトリ内の各ファイルは`main`関数を含み、スタンドアロンの実行可能なROS 2ノードとしてビルドされます。ここにあるソースファイルは、個別のノードとして独立して機能します。
- **src/**: このディレクトリには、ライブラリやその他の支援機能のためのソースファイルを配置します。これらのソースファイルは、ビルドプロセス中にすべてのノードのソースファイルにリンクされ、共有機能やユーティリティ関数を提供します。
- **include/**: このディレクトリには、ライブラリまたはノードに依存するヘッダーファイルが含まれています。
- **msg/**: このディレクトリはカスタムメッセージファイル用で、ROS 2通信に使用される特定のデータ構造を定義します。
- **srv/**: このディレクトリはカスタムサービスファイル用で、リクエストと応答で特定のデータを交換するサービスを定義します。
- **action/**: このディレクトリはカスタムアクションファイル用で、アクション通信のための目標、フィードバック、結果を定義します。

### 自動インターフェース生成

`ROS2_autoGenerateInterfaces()`関数を使用することで、メッセージ、サービス、アクションファイルを自動的に生成できます。以下は、`std_msgs`を依存関係として指定し、カスタムインターフェースを生成する例です。

```cmake
ROS2_autoGenerateInterfaces(std_msgs)
```

### 自動ノードビルド

`ROS2_autoBuildNodes()`関数は、指定された依存ライブラリに基づいてノードを自動的にビルドします。以下は、`rclcpp`と`std_msgs`を依存ライブラリとして指定する例です。

```cmake
ROS2_autoBuildNodes(rclcpp std_msgs)
```

### ライセンス
このプロジェクトは[MITライセンス](LICENSE)の下でライセンスされています。
