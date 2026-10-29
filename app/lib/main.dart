// PPG Watch 工具 —— 用藍牙連自製壓力手錶,同步時間、調整壓力判定線、看狀態。Android 版 (2026-10-29 決定不做 iOS)。
// 協定與韌體 firmware/src/ble_link.h 一致:服務 7a5e0001-…;指令寫 …0002 (一行一個、結尾 \n);狀態由 …0003 通知,一行
//   S,ms,score,thr,state,nRR,har,rec,fast,hr,worn,gIRcorr,vbat,chg,alt,pct
import 'dart:async';
import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:permission_handler/permission_handler.dart';

const svcUuid = '7a5e0001-8f2b-4c1d-9e3a-0c1f2a3b4c5d';
const cmdUuid = '7a5e0002-8f2b-4c1d-9e3a-0c1f2a3b4c5d';
const statusUuid = '7a5e0003-8f2b-4c1d-9e3a-0c1f2a3b4c5d';

void main() => runApp(const WatchToolApp());

class WatchToolApp extends StatelessWidget {
  const WatchToolApp({super.key});
  @override
  Widget build(BuildContext context) => MaterialApp(
        title: 'PPG Watch 工具',
        theme: ThemeData(colorSchemeSeed: const Color(0xFF2E7D32), useMaterial3: true),
        darkTheme: ThemeData(colorSchemeSeed: const Color(0xFF2E7D32), brightness: Brightness.dark, useMaterial3: true),
        home: const HomePage(),
      );
}

/// 手錶狀態列的一行
class WatchStatus {
  final int ms;
  final double score, thr;
  final int state, nRR, hr, worn, chg, pct;
  final String har;
  final bool rec, fast;
  final double vbat;
  final double? alt;
  final DateTime at;
  WatchStatus(this.ms, this.score, this.thr, this.state, this.nRR, this.har, this.rec, this.fast, this.hr, this.worn, this.vbat, this.chg, this.alt, this.pct, this.at);

  static WatchStatus? parse(String line) {
    final f = line.trim().split(',');
    if (f.length < 15 || f[0] != 'S') return null;
    int i(int k, [int d = 0]) => int.tryParse(f[k]) ?? d;
    double dd(int k) => double.tryParse(f[k]) ?? 0;
    try {
      return WatchStatus(i(1), dd(2), dd(3), i(4, -2), i(5), f[6], f[7] == '1', f[8] == '1', i(9), i(10, 1), dd(12), i(13),
          f.length > 14 ? double.tryParse(f[14]) : null, f.length > 15 ? i(15, -1) : -1, DateTime.now());
    } catch (_) {
      return null;
    }
  }

  String get label {
    if (worn == 0) return 'OFF WRIST';
    switch (state) {
      case 1:
        return 'STRESS';
      case 0:
        return 'CALM';
      case -1:
        return 'NO SIGNAL';
      default:
        return 'WAIT';
    }
  }
}

class HomePage extends StatefulWidget {
  const HomePage({super.key});
  @override
  State<HomePage> createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> {
  BluetoothAdapterState adapter = BluetoothAdapterState.unknown;
  final List<ScanResult> found = [];
  bool scanning = false, connecting = false, autoSync = true;
  BluetoothDevice? dev;
  BluetoothCharacteristic? cmd, statusCh;
  WatchStatus? status;
  String rawStatus = '', lastLine = '', error = '';
  DateTime? lastSync;
  final thrCtl = TextEditingController();
  final cmdCtl = TextEditingController();
  final subs = <StreamSubscription>[];
  Timer? ticker;

  @override
  void initState() {
    super.initState();
    subs.add(FlutterBluePlus.adapterState.listen((s) => setState(() => adapter = s)));
    subs.add(FlutterBluePlus.scanResults.listen((rs) => setState(() {
          found
            ..clear()
            ..addAll(rs);
        })));
    subs.add(FlutterBluePlus.isScanning.listen((s) => setState(() => scanning = s)));
    ticker = Timer.periodic(const Duration(seconds: 1), (_) => mounted ? setState(() {}) : null);
  }

  @override
  void dispose() {
    for (final s in subs) {
      s.cancel();
    }
    ticker?.cancel();
    thrCtl.dispose();
    cmdCtl.dispose();
    super.dispose();
  }

  bool get connected => dev != null && cmd != null;

  Future<bool> _permissions() async {
    // Android 12+ 要 SCAN/CONNECT;11 以下掃描要定位 (permission_handler 會依版本只問需要的)
    final r = await [Permission.bluetoothScan, Permission.bluetoothConnect, Permission.locationWhenInUse].request();
    final ok = (r[Permission.bluetoothScan]?.isGranted ?? false) && (r[Permission.bluetoothConnect]?.isGranted ?? false);
    if (!ok) setState(() => error = '沒有藍牙權限,到系統設定開啟後再試');
    return ok;
  }

  Future<void> scan() async {
    if (!await _permissions()) return;
    setState(() {
      error = '';
      found.clear();
    });
    try {
      await FlutterBluePlus.startScan(withServices: [Guid(svcUuid)], timeout: const Duration(seconds: 8));
    } catch (e) {
      setState(() => error = '掃描失敗:$e');
    }
  }

  Future<void> connect(BluetoothDevice d) async {
    await FlutterBluePlus.stopScan();
    setState(() {
      connecting = true;
      error = '';
    });
    try {
      await d.connect(timeout: const Duration(seconds: 12));
      await d.requestMtu(247);
      final svcs = await d.discoverServices();
      final svc = svcs.firstWhere((s) => s.uuid.str128.toLowerCase() == svcUuid, orElse: () => throw '這個裝置沒有手錶的服務');
      BluetoothCharacteristic? c, st;
      for (final ch in svc.characteristics) {
        final u = ch.uuid.str128.toLowerCase();
        if (u == cmdUuid) c = ch;
        if (u == statusUuid) st = ch;
      }
      if (c == null || st == null) throw '服務不完整 (缺指令或狀態特徵值)';
      await st.setNotifyValue(true);
      subs.add(st.onValueReceived.listen(_onStatus));
      subs.add(d.connectionState.listen((s) {
        if (s == BluetoothConnectionState.disconnected && mounted) {
          setState(() {
            dev = null;
            cmd = null;
            statusCh = null;
            error = '手錶已斷線';
          });
        }
      }));
      setState(() {
        dev = d;
        cmd = c;
        statusCh = st;
      });
      if (autoSync) await syncTime();
    } catch (e) {
      setState(() => error = '連線失敗:$e');
      try {
        await d.disconnect();
      } catch (_) {}
    } finally {
      if (mounted) setState(() => connecting = false);
    }
  }

  Future<void> disconnect() async {
    final d = dev;
    setState(() {
      dev = null;
      cmd = null;
      statusCh = null;
    });
    try {
      await d?.disconnect();
    } catch (_) {}
  }

  void _onStatus(List<int> v) {
    final line = String.fromCharCodes(v);
    final s = WatchStatus.parse(line);
    if (!mounted) return;
    setState(() {
      rawStatus = line.trim();
      if (s != null) {
        status = s;
        if (thrCtl.text.isEmpty) thrCtl.text = s.thr.toStringAsFixed(2);
      }
    });
  }

  Future<void> send(String c) async {
    final ch = cmd;
    if (ch == null) return;
    try {
      await ch.write(Uint8List.fromList('$c\n'.codeUnits), withoutResponse: true);
      setState(() {
        lastLine = c;
        error = '';
      });
    } catch (e) {
      setState(() => error = '送出失敗:$e');
    }
  }

  Future<void> syncTime() async {
    final now = DateTime.now();
    await send('TIME,${now.millisecondsSinceEpoch ~/ 1000},${now.timeZoneOffset.inMinutes}');
    setState(() => lastSync = now);
  }

  Future<void> applyThr() async {
    final v = double.tryParse(thrCtl.text.trim());
    if (v == null) {
      setState(() => error = '判定線要是數字,例如 0.5 或 -1.2');
      return;
    }
    await send('THR,${v.toStringAsFixed(3)}');
  }

  Future<void> resetThr() async {
    final ok = await showDialog<bool>(
          context: context,
          builder: (c) => AlertDialog(
            title: const Text('重設判定線?'),
            content: const Text('會刪掉手錶上存的個人判定線,回到模型預設值 (CNN 為 0)。'),
            actions: [TextButton(onPressed: () => Navigator.pop(c, false), child: const Text('取消')), FilledButton(onPressed: () => Navigator.pop(c, true), child: const Text('重設'))],
          ),
        ) ??
        false;
    if (ok) {
      await send('THR,RESET');
      thrCtl.clear();
    }
  }

  void nudge(double d) {
    final v = (double.tryParse(thrCtl.text.trim()) ?? status?.thr ?? 0) + d;
    thrCtl.text = v.toStringAsFixed(2);
  }

  @override
  Widget build(BuildContext context) {
    final s = status;
    final stale = s != null && DateTime.now().difference(s.at).inSeconds > 5;
    final cs = Theme.of(context).colorScheme;
    return Scaffold(
      appBar: AppBar(title: const Text('PPG Watch 工具')),
      body: ListView(padding: const EdgeInsets.all(12), children: [
        // ---- 連線 ----
        Card(
          child: Padding(
            padding: const EdgeInsets.all(12),
            child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
              Row(children: [
                Icon(connected ? Icons.bluetooth_connected : Icons.bluetooth, color: connected ? cs.primary : null),
                const SizedBox(width: 8),
                Expanded(child: Text(connected ? '已連線:${dev!.platformName.isEmpty ? dev!.remoteId.str : dev!.platformName}' : connecting ? '連線中…' : '未連線', style: Theme.of(context).textTheme.titleMedium)),
                if (connected) OutlinedButton(onPressed: disconnect, child: const Text('中斷')),
              ]),
              if (adapter != BluetoothAdapterState.on) Padding(padding: const EdgeInsets.only(top: 6), child: Text('藍牙狀態:${adapter.name}(請先開啟手機藍牙)', style: TextStyle(color: cs.error))),
              if (!connected) ...[
                const SizedBox(height: 8),
                Row(children: [
                  FilledButton.icon(onPressed: scanning || connecting ? null : scan, icon: const Icon(Icons.search), label: Text(scanning ? '掃描中…' : '掃描')),
                  const SizedBox(width: 12),
                  if (scanning) const SizedBox(width: 18, height: 18, child: CircularProgressIndicator(strokeWidth: 2)),
                ]),
                for (final r in found)
                  ListTile(
                    dense: true,
                    leading: const Icon(Icons.watch),
                    title: Text(r.device.platformName.isEmpty ? '(無名稱)' : r.device.platformName),
                    subtitle: Text('${r.device.remoteId.str}  RSSI ${r.rssi}'),
                    trailing: const Text('連線'),
                    onTap: connecting ? null : () => connect(r.device),
                  ),
                if (!scanning && found.isEmpty) const Padding(padding: EdgeInsets.only(top: 8), child: Text('按「掃描」找手錶(手錶螢幕亮著、距離 1 公尺內;只列出有本服務的裝置)。')),
              ],
              if (error.isNotEmpty) Padding(padding: const EdgeInsets.only(top: 6), child: Text(error, style: TextStyle(color: cs.error))),
            ]),
          ),
        ),
        // ---- 狀態 ----
        Card(
          child: Padding(
            padding: const EdgeInsets.all(12),
            child: s == null
                ? const Text('連上手錶後,這裡每秒顯示手錶的判定狀態。')
                : Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                    Row(crossAxisAlignment: CrossAxisAlignment.end, children: [
                      Text(s.label, style: TextStyle(fontSize: 30, fontWeight: FontWeight.bold, color: s.label == 'STRESS' ? Colors.red : s.label == 'CALM' ? Colors.green : cs.outline)),
                      const SizedBox(width: 12),
                      Text('${s.score.toStringAsFixed(2)} / 判定線 ${s.thr.toStringAsFixed(2)}', style: Theme.of(context).textTheme.bodyLarge),
                    ]),
                    const SizedBox(height: 6),
                    Wrap(spacing: 14, runSpacing: 4, children: [
                      Text('心率 ${s.hr} bpm'),
                      Text(s.worn == 1 ? '配戴中' : '未配戴(暫停判定)'),
                      Text('活動 ${s.har}'),
                      Text(s.pct >= 0 ? '電量 ${s.pct}%${s.chg == 1 ? ' 充電中' : ''}' : '電壓 ${s.vbat.toStringAsFixed(2)} V'),
                      if (s.fast) const Text('校正模式'),
                      if (s.rec) const Text('錄製標記中'),
                    ]),
                    const SizedBox(height: 4),
                    Text(stale ? '最後一筆是 ${DateTime.now().difference(s.at).inSeconds} 秒前(手錶沒在送?)' : '每秒更新', style: TextStyle(color: stale ? cs.error : cs.outline, fontSize: 12)),
                  ]),
          ),
        ),
        // ---- 時間 ----
        Card(
          child: Padding(
            padding: const EdgeInsets.all(12),
            child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
              Text('時間', style: Theme.of(context).textTheme.titleMedium),
              const SizedBox(height: 4),
              const Text('手錶沒有時鐘電池,開機後要由手機對時;沒對時螢幕顯示 --:--。'),
              Row(children: [
                FilledButton.icon(onPressed: connected ? syncTime : null, icon: const Icon(Icons.schedule), label: const Text('同步時間')),
                const SizedBox(width: 12),
                Expanded(child: Text(lastSync == null ? '尚未同步' : '上次同步 ${_hms(lastSync!)}')),
              ]),
              CheckboxListTile(
                contentPadding: EdgeInsets.zero,
                dense: true,
                value: autoSync,
                onChanged: (v) => setState(() => autoSync = v ?? true),
                title: const Text('連上手錶時自動同步'),
              ),
            ]),
          ),
        ),
        // ---- 判定線 ----
        Card(
          child: Padding(
            padding: const EdgeInsets.all(12),
            child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
              Text('壓力判定線', style: Theme.of(context).textTheme.titleMedium),
              const SizedBox(height: 4),
              Text(s == null ? '連上後顯示手錶目前的判定線。' : '手錶目前:${s.thr.toStringAsFixed(3)}(分數 > 判定線 = STRESS;CNN 的分數是 logit,預設 0)'),
              Row(children: [
                IconButton(onPressed: () => nudge(-0.1), icon: const Icon(Icons.remove_circle_outline)),
                SizedBox(
                  width: 120,
                  child: TextField(
                    controller: thrCtl,
                    keyboardType: const TextInputType.numberWithOptions(decimal: true, signed: true),
                    textAlign: TextAlign.center,
                    decoration: const InputDecoration(labelText: '新判定線', isDense: true, border: OutlineInputBorder()),
                  ),
                ),
                IconButton(onPressed: () => nudge(0.1), icon: const Icon(Icons.add_circle_outline)),
                const SizedBox(width: 8),
                FilledButton(onPressed: connected ? applyThr : null, child: const Text('套用判定線')),
              ]),
              TextButton.icon(onPressed: connected ? resetThr : null, icon: const Icon(Icons.restart_alt), label: const Text('重設判定線(回模型預設)')),
            ]),
          ),
        ),
        // ---- 進階 ----
        Card(
          child: ExpansionTile(
            title: const Text('進階:送指令'),
            childrenPadding: const EdgeInsets.fromLTRB(12, 0, 12, 12),
            children: [
              Wrap(spacing: 6, children: [
                for (final q in const ['STAT', 'SLEEP,0', 'SLEEP,15', 'LIVE,1', 'LIVE,0', 'MODEL,1', 'MODEL,0', 'REBOOT'])
                  ActionChip(label: Text(q), onPressed: connected ? () => send(q) : null),
              ]),
              Row(children: [
                Expanded(child: TextField(controller: cmdCtl, decoration: const InputDecoration(labelText: '指令(例如 SRATE,5)', isDense: true, border: OutlineInputBorder()))),
                const SizedBox(width: 8),
                FilledButton(onPressed: connected ? () => send(cmdCtl.text.trim()) : null, child: const Text('送出')),
              ]),
              const SizedBox(height: 6),
              Align(alignment: Alignment.centerLeft, child: Text('回應印在手錶的序列埠;狀態列的變化看得到。最後送出:$lastLine', style: TextStyle(color: cs.outline, fontSize: 12))),
              if (rawStatus.isNotEmpty) Align(alignment: Alignment.centerLeft, child: SelectableText('狀態列原文:$rawStatus', style: const TextStyle(fontSize: 11, fontFamily: 'monospace'))),
            ],
          ),
        ),
      ]),
    );
  }

  static String _hms(DateTime t) => '${t.hour.toString().padLeft(2, '0')}:${t.minute.toString().padLeft(2, '0')}:${t.second.toString().padLeft(2, '0')}';
}
