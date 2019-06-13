# choreonoid_inverse_dynamic_test
コレオノイドの逆運動学計算のテスト。
#テスト方法

```bash
$ cmake .;make;./1link
```

#テスト内容
モデルは1linkbot.yamlにて定義した。単純な1linkのみのモデルを用いた。
関節角度を0[deg]から90[deg]へ2秒間で移動するように補間し、
この軌道を実現するためのトルクをchoreonoidのライブラリと自力でラグランジュ法でそれぞれ計算し、トルクを比較する。

全時刻において両者の差が1e-8以下ならテストは成功とする。ログファイルはref_tau.logに出力される。

ログのデータは、choreonoidで計算されたトルク、ラグランジュ法で計算されたトルク、関節角度[rad]の順で並んでいる。

ただし、現在バグがあるようで、テストは失敗する。

[issue](https://discourse.choreonoid.org/t/topic/292)
このissueで提案した修正を行うことでテストは通る。
