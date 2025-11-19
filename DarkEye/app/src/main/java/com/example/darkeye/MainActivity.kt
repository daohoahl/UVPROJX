package com.example.darkeye

import android.graphics.Color
import android.os.Bundle
import android.view.TextureView
import android.view.View
import android.widget.*
import androidx.appcompat.app.AppCompatActivity

class MainActivity : AppCompatActivity() {

    // Views
    private lateinit var videoView: TextureView
    private lateinit var placeholderLayout: LinearLayout
    private lateinit var tvConnectionStatus: TextView
    private lateinit var tvBattery: TextView
    private lateinit var ivBattery: ImageView
    private lateinit var alertPanel: LinearLayout
    private lateinit var tvAlertMessage: TextView
    private lateinit var tvNightVision: TextView

    // Control buttons - QUAN TRỌNG
    private lateinit var btnForward: Button
    private lateinit var btnBackward: Button
    private lateinit var btnLeft: Button
    private lateinit var btnRight: Button
    private lateinit var btnStop: Button
    private lateinit var btnLeftInner: Button
    private lateinit var btnRightInner: Button

    // Secondary buttons - PHỤ (giờ là ImageButton)
    private lateinit var btnNightVision: ImageButton  // Đã đổi thành ImageButton
    private lateinit var btnCapture: ImageButton      // Đã đổi thành ImageButton
    private lateinit var btnRecord: ImageButton       // Đã đổi thành ImageButton
    private lateinit var btnConnect: ImageView
    private lateinit var btnMenu: ImageView
    private lateinit var btnCloseAlert: ImageButton

    // UI States
    private var isConnected = false
    private var isRecording = false
    private var isNightVisionOn = false
    private var batteryLevel = 85

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        initializeViews()
        setupUI()
        setupClickListeners()
        simulateUIStates()
    }

    private fun initializeViews() {
        // Video & Status views
        videoView = findViewById(R.id.videoView)
        placeholderLayout = findViewById(R.id.placeholderLayout)
        tvConnectionStatus = findViewById(R.id.tvConnectionStatus)
        tvBattery = findViewById(R.id.tvBattery)
        ivBattery = findViewById(R.id.ivBattery)
        alertPanel = findViewById(R.id.alertPanel)
        tvAlertMessage = findViewById(R.id.tvAlertMessage)
        tvNightVision = findViewById(R.id.tvNightVision)

        // PRIMARY Controls - ĐIỀU KHIỂN QUAN TRỌNG
        btnForward = findViewById(R.id.btnForward)
        btnBackward = findViewById(R.id.btnBackward)
        btnLeft = findViewById(R.id.btnLeft)
        btnRight = findViewById(R.id.btnRight)
        btnStop = findViewById(R.id.btnStop)
        btnLeftInner = findViewById(R.id.btnLeftInner)
        btnRightInner = findViewById(R.id.btnRightInner)

        // SECONDARY Controls - CHỨC NĂNG PHỤ (ImageButton)
        btnNightVision = findViewById(R.id.btnNightVision)
        btnCapture = findViewById(R.id.btnCapture)
        btnRecord = findViewById(R.id.btnRecord)
        btnConnect = findViewById(R.id.btnConnect)
        btnMenu = findViewById(R.id.btnMenu)
        btnCloseAlert = findViewById(R.id.btnCloseAlert)
    }

    private fun setupUI() {
        // Ưu tiên video area
        placeholderLayout.visibility = View.VISIBLE

        // Status nhỏ gọn
        tvConnectionStatus.setTextColor(Color.parseColor("#00FF88"))
        tvBattery.setTextColor(Color.WHITE)
        updateBatteryDisplay()
    }

    private fun setupClickListeners() {
        // PRIMARY Controls - ĐIỀU KHIỂN CHÍNH (lớn)
        btnForward.setOnClickListener { showToast("TIẾN") }
        btnBackward.setOnClickListener { showToast("LÙI") }
        btnLeft.setOnClickListener { showToast("TRÁI") }
        btnRight.setOnClickListener { showToast("PHẢI") }
        btnStop.setOnClickListener { showToast("DỪNG") }
        btnLeftInner.setOnClickListener { showToast("TRÁI") }
        btnRightInner.setOnClickListener { showToast("PHẢI") }

        // SECONDARY Controls - CHỨC NĂNG PHỤ (hình ảnh)
        btnNightVision.setOnClickListener { toggleNightVision() }
        btnCapture.setOnClickListener { showToast("CHỤP ẢNH") }
        btnRecord.setOnClickListener { toggleRecording() }
        btnConnect.setOnClickListener { showToast("KẾT NỐI") }
        btnMenu.setOnClickListener { showToast("MENU") }
        btnCloseAlert.setOnClickListener {
            alertPanel.visibility = View.GONE
        }

        // Demo alerts
        setupDemoFeatures()
    }

    private fun setupDemoFeatures() {
        // Long press để test alert
        btnForward.setOnLongClickListener {
            showAlert("PHÁT HIỆN CHUYỂN ĐỘNG!", "MOTION")
            true
        }

        btnBackward.setOnLongClickListener {
            showAlert("CẢNH BÁO HỎA HOẠN!", "FIRE")
            true
        }
    }

    private fun toggleNightVision() {
        isNightVisionOn = !isNightVisionOn
        if (isNightVisionOn) {
            // Đổi màu nền khi bật chế độ đêm
            btnNightVision.setBackgroundColor(Color.parseColor("#4A4AFF"))
            tvNightVision.visibility = View.VISIBLE
            showToast("BẬT CHẾ ĐỘ ĐÊM")
        } else {
            // Trở về màu nền mặc định
            btnNightVision.setBackgroundResource(R.drawable.btn_control_background)
            tvNightVision.visibility = View.GONE
            showToast("TẮT CHẾ ĐỘ ĐÊM")
        }
    }

    private fun toggleRecording() {
        isRecording = !isRecording
        if (isRecording) {
            // Đổi màu nền khi đang ghi
            btnRecord.setBackgroundColor(Color.parseColor("#FF4444"))
            showToast("BẮT ĐẦU GHI HÌNH")
        } else {
            // Trở về màu nền mặc định
            btnRecord.setBackgroundResource(R.drawable.btn_control_background)
            showToast("DỪNG GHI HÌNH")
        }
    }

    private fun updateBatteryDisplay() {
        tvBattery.text = "$batteryLevel%"
        when {
            batteryLevel > 70 -> {
                ivBattery.setImageResource(R.drawable.ic_battery_full)
                ivBattery.setColorFilter(Color.parseColor("#00FF88"))
            }
            batteryLevel > 30 -> {
                ivBattery.setImageResource(R.drawable.ic_battery_medium)
                ivBattery.setColorFilter(Color.parseColor("#FFA500"))
            }
            else -> {
                ivBattery.setImageResource(R.drawable.ic_battery_low)
                ivBattery.setColorFilter(Color.parseColor("#FF4444"))
            }
        }
    }

    private fun showAlert(message: String, alertType: String) {
        tvAlertMessage.text = message
        when (alertType) {
            "MOTION" -> alertPanel.setBackgroundColor(Color.parseColor("#FF6B00"))
            "FIRE" -> alertPanel.setBackgroundColor(Color.parseColor("#FF4444"))
        }
        alertPanel.visibility = View.VISIBLE
    }

    private fun showToast(message: String) {
        Toast.makeText(this, message, Toast.LENGTH_SHORT).show()
    }

    private fun simulateUIStates() {
        updateConnectionStatus(true)
        updateBatteryLevel(85)
    }

    private fun updateConnectionStatus(connected: Boolean) {
        isConnected = connected
        if (connected) {
            tvConnectionStatus.text = "Đã kết nối"
            tvConnectionStatus.setTextColor(Color.parseColor("#00FF88"))
            btnConnect.setColorFilter(Color.parseColor("#00FF88"))
            placeholderLayout.visibility = View.GONE
        } else {
            tvConnectionStatus.text = "Mất kết nối"
            tvConnectionStatus.setTextColor(Color.parseColor("#FF4444"))
            btnConnect.setColorFilter(Color.parseColor("#CCCCCC"))
            placeholderLayout.visibility = View.VISIBLE
        }
    }

    private fun updateBatteryLevel(level: Int) {
        batteryLevel = level
        updateBatteryDisplay()
    }
}