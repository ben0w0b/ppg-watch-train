// 不靠平台的測試:狀態列解析與標籤 (Android / iOS 共用同一份程式,這裡跑在 JVM/VM 上)。
import 'package:flutter_test/flutter_test.dart';
import 'package:watch_tool/main.dart';

void main() {
  test('parse a 16-field status line', () {
    final s = WatchStatus.parse('S,123456,0.71,0.55,1,0,static,1,0,92,1,0.00,0.00,0,NaN,87');
    expect(s, isNotNull);
    expect(s!.score, closeTo(0.71, 1e-9));
    expect(s.thr, closeTo(0.55, 1e-9));
    expect(s.state, 1);
    expect(s.har, 'static');
    expect(s.hr, 92);
    expect(s.worn, 1);
    expect(s.pct, 87);
    expect(s.fast, isFalse);
    expect(s.label, 'STRESS');
  });
  test('labels', () {
    expect(WatchStatus.parse('S,1,-1.2,0.0,0,0,walk,0,0,70,1,0,3.9,0,NaN,50')!.label, 'CALM');
    expect(WatchStatus.parse('S,1,0,0,-1,0,--,0,0,0,1,0,3.9,0,NaN,50')!.label, 'NO SIGNAL');
    expect(WatchStatus.parse('S,1,0,0,-2,0,--,0,0,0,1,0,3.9,0,NaN,50')!.label, 'WAIT');
    expect(WatchStatus.parse('S,1,0.9,0.5,1,0,--,0,0,0,0,0,3.9,0,NaN,50')!.label, 'OFF WRIST');
  });
  test('old 15-field line and garbage', () {
    final s = WatchStatus.parse('S,1,0.1,0.2,0,5,sit,0,1,80,1,0.5,4.1,1,2.5');
    expect(s, isNotNull);
    expect(s!.pct, -1);
    expect(s.fast, isTrue);
    expect(s.alt, closeTo(2.5, 1e-9));
    expect(WatchStatus.parse('hello'), isNull);
    expect(WatchStatus.parse('S,1,2'), isNull);
  });
}
