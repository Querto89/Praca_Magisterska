# Digital Twin napędu DC — optymalizacja DP + detekcja ataku FDI

Refaktoryzacja projektu pod kątem pracy magisterskiej o **cyberbezpieczeństwie
i optymalizacji**.

---

## 1. Co było nie tak w wersji pierwotnej

| # | Problem | Skutek | Naprawa |
|---|---------|--------|---------|
| 1 | `Solver='ode1'`, `FixedStep=0.05 s` | Wartości własne obiektu to −20±20j, więc \|1+λ·dt\| = **1.0** — jawny Euler zamieniał stabilny silnik w **niegasnący oscylator numeryczny** (4-krokowy cykl, rozbieżność ~32 rad/s) | `ode4`, krok `dt/50 = 1 ms` |
| 2 | Model i "obiekt" miały **te same parametry** | Residuum bez ataku = 0 → dobór progu trywialny, brak wartości naukowej | `p.plant = p.nom .* p.mismatch` (1–3 % błędu identyfikacji) |
| 3 | **Ataku FDI nie było w ogóle** | `run_simulink` szukał `outAttack_Detected`, którego nie ma w `.slx`; cicho zwracał zera | Atak wstrzykiwany w torze pomiarowym (`measurement_channel.m`) |
| 4 | `run_simulink` czytał sygnał `E_dp` | Taki blok **nie istnieje** w modelu — energia nigdy nie była liczona | Energia liczona w MATLAB: `E = Σ u·i·dt` |
| 5 | Digital Twin jako model **swobodny** | Przy 5 % błędu `Ra` rozjazd w stanie ustalonym ~1.3 rad/s — **większy niż typowy atak**; atak o przeciwnym znaku *zmniejszał* residuum i stawał się niewidoczny | Predyktor **jednokrokowy** + test **dwustronny** |
| 6 | `p.residual_threshold = 1.0` | Próg wybrany arbitralnie | Kalibracja Monte Carlo: `próg = k_σ · σ(k)` wokół `μ(k)` |
| 7 | Asercja w `Matlab_Test.m` | Zakładała `Uopt` 2-D, a solver zwraca 3-D — test zawsze by padał | Poprawione wymiary `[Ni × Nw × N]` |
| 8 | DP: „najbliższy węzeł” + potrójna pętla | Błąd kwantyzacji podnosił residuum; wolne | Interpolacja biliniowa + wektoryzacja |
| 9 | Sterowanie w pętli **otwartej** | Atak nie wpływał na regulator → nie dało się badać związku bezpieczeństwo↔optymalność | Pętla zamknięta: DP czyta **pomiar** |

> **Uwaga:** jeden błąd znalazłem też we własnej pierwszej wersji detektora —
> CUSUM liczony jako `S = max(0, S + |z| − k)` **dryfuje bez ataku**
> (bo `E[|z|] ≈ 0.8 > k = 0.5`) i po ~17 krokach daje pewny fałszywy alarm.
> Poprawna wersja jest **dwustronna i na residuum ze znakiem** (`S⁺`, `S⁻`).

---

## 2. Architektura

```
   OBIEKT RZECZYWISTY  (Simulink, parametry p.plant)
          │  x(k) = [i_a; ω]
          ▼
   CZUJNIKI (szum σ_ω, σ_i)
          ▼
   ══ ATAK FDI ══  bias / ramp / scaling / replay      ← kanał telemetrii
          │  y(k)
          ├──────────────► DIGITAL TWIN (p.nom, predykcja 1-krokowa)
          │                      │  r(k) = ω_meas − ω_pred
          │                      ▼
          │                 DETEKTOR: |z| > k_σ  ∨  CUSUM  → ALARM
          ▼
   REGULATOR DP (tablica Uopt) ──► u(k) ──► OBIEKT
```

**Atak siedzi na kanale pomiarowym, nie w fizyce silnika** — to definicja
FDI (w odróżnieniu od uszkodzenia fizycznego / awarii aktuatora).

---

## 3. Pliki

| Plik | Rola |
|------|------|
| `params.m` | Cała konfiguracja: `p.nom` vs `p.plant`, czujniki, atak, detektor |
| `motor_model.m` | Równania silnika + dyskretyzacja ZOH (jedno źródło prawdy) |
| `dp_solver.m` | Programowanie dynamiczne (wektoryzowane, interpolacja biliniowa) |
| `plant_matlab.m` | Obiekt — drobny krok, szybki (kampanie Monte Carlo) |
| `plant_simulink.m` | Obiekt — `system_model.slx`, sterowanie krok po kroku |
| `verify_plant.m` | Walidacja: Simulink vs MATLAB dla **tych samych** parametrów |
| `measurement_channel.m` | Szum czujników + wstrzyknięcie ataku FDI |
| `detector.m` | Residuum predyktora + test dwustronny + CUSUM |
| `calibrate_detector.m` | Kalibracja `μ(k)`, `σ(k)` i progu na przebiegach bez ataku |
| `simulate_system.m` | Silnik symulacji w pętli zamkniętej |
| `attack_campaign.m` | Kampania Monte Carlo → tabela `P_D`, opóźnienie, energia |
| `plots_nominal.m` | DP vs baseline (jakość sterowania) |
| `plots_security.m` | Pomiar vs twin, residuum, statystyka decyzyjna, alarm |
| `Matlab_Test.m` | 9 testów jednostkowych |
| `main.m` | Uruchamia całość |

---

## 4. Uruchomienie

```matlab
main          % pełny przebieg (DP → walidacja → kalibracja → atak → kampania)
Matlab_Test   % testy jednostkowe
```

Bez Simulinka: `p.use_simulink = false` w `params.m` (obiekt liczony w MATLAB).
`main.m` i tak przełącza się automatycznie, jeśli Simulink zawiedzie.

---

## 5. Wyniki walidacji numerycznej

Cały łańcuch (DP + obiekt + czujniki + detektor) przeliczyłem niezależnie,
żeby dobór parametrów nie był zgadywaniem:

- **Obciążenie od niedopasowania:** `μ ≈ −0.20 rad/s` (deterministyczne → kalibrowane)
- **Szum residuum:** `σ ≈ 0.0224 rad/s`
- **Próg (5σ):** `≈ 0.112 rad/s`
- **FAR:** 0 / 300 przebiegów (próg + persystencja + CUSUM)
- **Punkt pracy bez ataku:** ω_end ≈ 9.70 rad/s, E ≈ 19.6 J

### Granica skrytości ataku (bias)

| Amplituda [rad/s] | P_D |
|---|---|
| 0.005 | 0 % |
| 0.01 | 0 % |
| 0.02 | 23 % |
| 0.05 | 100 % |

Przejście 0 % → 100 % zachodzi **w okolicy odchylenia szumu czujnika**
(σ_ω = 0.02 rad/s), czyli ok. **0.1–0.2 % prędkości roboczej**.
To jest ilościowa **granica skrytości** — dobry wynik do pracy.

---

## 6. Wątki do rozdziału dyskusyjnego

1. **Kompromis czułość ↔ fałszywe alarmy** — przemiataj `k_sigma` i `cusum_h`,
   narysuj ROC. Próg wynika z założonego FAR, nie z „oka”.
2. **Wrażliwość na jakość identyfikacji** — zwiększ `p.mismatch`; podłoga
   residuum rośnie i granica skrytości przesuwa się w górę. Pokazuje, że
   **jakość Digital Twina = jakość detekcji**.
3. **Atak na fazę kalibracji** — jeśli atakujący jest obecny podczas uczenia,
   detektor „wchłania” atak jako normę (poisoning).
4. **Bezpieczeństwo ↔ optymalność** — kolumna `Energia_J` w kampanii pokazuje,
   ile atak kosztuje w sensie kryterium DP, nawet gdy jest wykryty.
5. **Replay** — teoretycznie najtrudniejszy do wykrycia; przy dłuższym stanie
   ustalonym staje się skryty (klasyczny argument za *watermarkingiem*).
