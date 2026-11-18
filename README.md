# Adaptive Multi-Drone Search & Terrain Mapping System 🛸🛰️

نظام متكامل للتخطيط الجغرافي التكيفي والتوجيه الذكي لأسراب الطائرات المسيرة متعددة المهام (`Adaptive Multi-UAV Mission Planning & Search-and-Rescue System`). يجمع النظام بين الهندسة الجيوديسية الحوسبية (`Computational GIS`)، وخوارزميات التحسين التوافقي الهجينة ذاتية التكيف (`Online RL-Guided Hybrid GA-PSO`)، والمحاكاة الفيزيائية في الزمن الحقيقي (`Real-Time Physics Simulator & WebSockets`)، ولوحات القيادة والتحكم التفاعلية عبر الخرائط الفضائية (`React + Google Maps API`).

---

## 🌟 الميزات الهندسية والمعمارية الرئيسية (Key Features)

1. **فضاء البحث الهجين المتكامل (`Unified Hybrid Graph-Grid Space`):**
   * دمج طبوغرافيا شبكة الطرق الحضرية المستخرجة من OpenStreetMap مع شبكة الخلايا الجيوديسية المتساوية المساحة (WGS-84).
   * تطبيق حواجز الاستبعاد المكاني الجيوديسي (`Spatial Exclusion Buffer`) لمنع ازدواجية مسح التقاطعات.
   * تقنية ضغط الخلايا بشرائح بوستروفيدون (`Boustrophedon Cellular Decomposition`) للتعامل مع المناطق الكبرى ($> 13\text{ km}^2$) بكفاءة زمنية فائقة ($< 0.8\text{s}$).

2. **محرك التحسين المقارن الثلاثي (`Tri-Algorithm Optimization Engine`):**
   * **Discrete PSO:** تحسين سرب الجسيمات المتقطع القابل للتوسع لأي حجم أسطول ($N \ge 1$).
   * **Modified GA:** الخوارزمية الوراثية المعدلة مع تقاطعات القطاعات المكانية والانتخاب المتوازن.
   * **Online RL-Guided Hybrid GA-PSO:** محرك هجين يدمج الخوارزميتين مع تعلّم معزز فوري بدون تدريب مسبق (`Zero Pre-Training Bellman Updates`) لتحقيق أفضل زمن إنجاز (`Makespan`).

3. **التخطيط متعدد الجولات والتغطية الفيزيائية الكاملة ($100\%$):**
   * تقطيع المهام الطويلة آلياً إلى دورات طيران متتالية (`Sorties`) عند استهلاك البطارية.
   * حقن نقاط العودة واستبدال البطارية والإقلاع الجديد (`SORTIE_RETURN_BASE`, `BATTERY_SWAP`, `SORTIE_RELAUNCH_BASE`).
   * خوارزمية فك تشابك المسارات داخل كل جولة (`Intra-Sortie 2-Opt + Or-Opt Untangling`) لمنع تقاطع خطوط الطيران.

4. **لوحة القيادة والتحكم في الزمن الحقيقي (`Mission Control HUD`):**
   * خريطة متعددة الطبقات بدقة الأقمار الصناعية مدعومة بـ Google Maps JavaScript API.
   * محاكاة فيزيائية 10Hz مع بث فوري عبر الـ WebSockets (`Telemetry Stream`).
   * نافذة التحليل الهندسي المقارن الفوري (`Computational Route Analytics & Tie-Aware Comparison`).

---

## 🔒 تأمين مفاتيح الـ API وإعداد الخرائط (API Keys & Maps Security)

> [!IMPORTANT]
> **قاعدة أمان صارمة:** لا تقم أبداً برفع ملفات `.env` أو مفتاح Google Maps API على مستودعات Git العامة أو الخاصة. يتم استبعاد ملفات `.env` تلقائياً عبر `.gitignore`.

### 1. إعداد مفتاح الخرائط للاستخدام المحلي:
1. انسخ ملف النموذج `frontend/.env.example` إلى `frontend/.env`:
   ```bash
   cp frontend/.env.example frontend/.env
   ```
2. افتح ملف `frontend/.env` وضع مفتاحك الخاص:
   ```env
   VITE_GOOGLE_MAPS_API_KEY=AIzaSy...your_actual_key_here
   VITE_API_BASE_URL=http://localhost:8000
   ```

### 2. حماية المفتاح من الحظر وسرقة الحصص (Google Cloud Console Best Practice):
لحماية مفتاحك من الاستخدام غير المصرح به وتجنب حظره أو استنفاد رصيدك:
* توجه إلى **[Google Cloud Console](https://console.cloud.google.com/) -> APIs & Services -> Credentials**.
* اختر المفتاح وقم بتفعيل **Application restrictions -> Websites (HTTP referrers)**.
* أضف النطاقات المحلية المصرح لها فقط:
  * `http://localhost:5173/*`
  * `http://127.0.0.1:5173/*`
* فعّل القيود على واجهات البرمجة المسموحة (**API restrictions**):
  * `Maps JavaScript API`
  * `Geocoding API`

### 3. الوضع التجريبي بدون مفتاح (`Offline Sandbox Mode`):
في حال تشغيل النظام دون توفير مفتاح Google Maps، سيعمل النظام تلقائياً في **وضع المحاكاة التفاعلية (`Interactive Search Space Sandbox Mode`)** الذي يتيح اختبار خوارزميات التقطيع والتوجيه والمحاكاة الفيزيائية كاملة دون الحاجة لخرائط خارجية.

---

## 🚀 التثبيت والتشغيل (Installation & Quickstart)

### المتطلبات الأساسية (Prerequisites):
* **Python 3.10+**
* **Node.js 18+** و **npm**

---

### أولاً: تشغيل الخادم الخلفي (Backend - FastAPI)

1. الانتقال إلى مجلد الخادم:
   ```bash
   cd backend
   ```

2. إنشاء البيئة الافتراضية وتثبيت الحزم:
   ```bash
   python -m venv venv
   # On Windows:
   .\venv\Scripts\activate
   # On Linux/macOS:
   source venv/bin/activate

   pip install -r requirements.txt
   ```

3. بدء تشغيل الخادم:
   ```bash
   python -m uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
   ```
   * واجهة توثيق الـ API التفاعلية: [http://localhost:8000/docs](http://localhost:8000/docs)
   * خادم الـ WebSockets: `ws://localhost:8000/ws/telemetry`

---

### ثانياً: تشغيل الواجهة الأمامية (Frontend - React + Vite)

1. الانتقال إلى مجلد الواجهة:
   ```bash
   cd frontend
   ```

2. تثبيت الحزم البرمجية:
   ```bash
   npm install
   ```

3. تجهيز ملف الإعدادات البيئية:
   ```bash
   # تأكد من وجود ملف .env بناءً على .env.example
   npm run dev
   ```
4. افتح المتصفح على: [http://localhost:5173](http://localhost:5173)

---

## 🏛️ البنية المعمارية البرمجية (System Architecture)

للاطلاع على المخططات الهندسية المعمارية الكاملة ومخططات صنف UML وتدفق البيانات بين الطبقات، راجع:
* **[مستند المعمارية الشامل (UML_ARCHITECTURE.md)](UML_ARCHITECTURE.md)**
* **[عارض المخططات التفاعلي (docs/uml_viewer.html)](docs/uml_viewer.html)**

```
Adaptive_multi_drones/
├── backend/
│   ├── app/
│   │   ├── api/v1/         # REST API Routers (grid, map, fleet, optimize, simulation)
│   │   ├── api/ws/         # Real-time WebSocket Telemetry Hub
│   │   ├── core/           # Session Epoch & Dynamic State Management
│   │   ├── fleet/          # Dynamic Zero-Hardcoding Fleet Registry
│   │   ├── gis/            # Geodesic Discretizer & Road Topology Extractor
│   │   ├── optimizers/     # PSO, GA, Online RL Hybrid GA-PSO, Fitness & Analytics
│   │   ├── physics/        # UAV Flight Physics & Energy Consumption Models
│   │   └── simulation/     # Multi-Drone Physics Engine & LiDAR Coverage Sim
│   ├── tests/              # End-to-End Stress & Verification Test Suites
│   └── requirements.txt
├── frontend/
│   ├── src/
│   │   ├── components/     # MultiLayerMissionMap, InteractiveMapSelector, Header, HUD
│   │   ├── pages/          # MissionControlDashboard
│   │   ├── services/       # WebSocket Telemetry Hook & API Client
│   │   └── types/          # TypeScript Type Definitions
│   ├── .env.example        # Environment Template
│   └── package.json
├── docs/                   # Interactive UML Viewer & Design Artifacts
├── UML_ARCHITECTURE.md     # Full System Architecture Specification
└── .gitignore              # Strict Security & Dependency Ignore Rules
```

---

## 📄 الترخيص (License)
هذا المشروع مرخص تحت رخصة **MIT License** - متاح للأغراض البحثية والأكاديمية والتطوير الحر.
