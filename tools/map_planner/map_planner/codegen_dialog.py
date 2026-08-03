"""Small PySide6 dialog for previewing and copying generated STM32 C code."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QApplication,
    QDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
)

from .codegen_c import (
    CodeGenerationMode,
    CodeGenerationError,
    default_task_function_name,
    generate_task_function,
    validate_plan_for_blocking_codegen,
)
from .models import Plan


class CodeGenerationDialog(QDialog):
    """Preview and copy one generated task function without touching the plan."""

    def __init__(self, plan: Plan, parent=None) -> None:  # type: ignore[no-untyped-def]
        super().__init__(parent)
        self.plan = plan
        self.mode = CodeGenerationMode.FEEDBACK
        self.generated_code = ""
        self._warnings: list[str] = []
        self.setWindowTitle("生成 STM32 业务函数")
        self.resize(820, 640)

        self.plan_name_edit = QLineEdit(plan.name)
        self.plan_name_edit.setReadOnly(True)
        self.function_name_edit = QLineEdit(default_task_function_name(plan.name))
        self.warning_label = QLabel()
        self.warning_label.setWordWrap(True)
        self.warning_label.setTextInteractionFlags(
            self.warning_label.textInteractionFlags()
            | Qt.TextInteractionFlag.TextSelectableByMouse
        )
        self.code_preview = QPlainTextEdit()
        self.code_preview.setReadOnly(True)
        self.code_preview.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)

        self.regenerate_button = QPushButton("重新生成")
        self.mode_button = QPushButton()
        self.copy_button = QPushButton("复制代码")
        self.close_button = QPushButton("关闭")
        self.regenerate_button.clicked.connect(self.regenerate)
        self.mode_button.clicked.connect(self.toggle_mode)
        self.copy_button.clicked.connect(self.copy_code)
        self.close_button.clicked.connect(self.close)

        form = QFormLayout()
        form.addRow("方案名称", self.plan_name_edit)
        form.addRow("函数名称", self.function_name_edit)
        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addWidget(self.warning_label)
        layout.addWidget(self.code_preview, 1)
        buttons = QHBoxLayout()
        buttons.addWidget(self.mode_button)
        buttons.addStretch()
        buttons.addWidget(self.regenerate_button)
        buttons.addWidget(self.copy_button)
        buttons.addWidget(self.close_button)
        layout.addLayout(buttons)
        self._update_mode_button()
        self.regenerate()

    def _update_mode_button(self) -> None:
        if self.mode is CodeGenerationMode.FEEDBACK:
            self.mode_button.setText("模式：严谨反馈")
            self.mode_button.setToolTip("切换为开环忽略结果模式")
        else:
            self.mode_button.setText("模式：开环忽略结果")
            self.mode_button.setToolTip("切换为严谨反馈模式")

    def toggle_mode(self) -> None:
        """Switch result handling for this preview without changing the plan."""

        self.mode = (CodeGenerationMode.OPEN_LOOP if self.mode is CodeGenerationMode.FEEDBACK
                     else CodeGenerationMode.FEEDBACK)
        self._update_mode_button()
        self.regenerate()

    def _mode_message(self) -> str:
        if self.mode is CodeGenerationMode.FEEDBACK:
            return "严谨反馈：运动未到达时取消后续步骤并退出函数。"
        return "开环忽略结果：顺序执行阻塞运动调用，不检查到达、超时或取消状态。"

    def regenerate(self) -> None:
        """Regenerate preview while preserving the last valid code on errors."""

        try:
            self._warnings = validate_plan_for_blocking_codegen(self.plan)
            code = generate_task_function(self.plan, self.function_name_edit.text(), self.mode)
        except CodeGenerationError as error:
            self.warning_label.setText(str(error))
            self.copy_button.setEnabled(False)
            return

        self.generated_code = code
        self.code_preview.setPlainText(code)
        self.warning_label.setText("\n".join([self._mode_message(), *self._warnings]))
        self.copy_button.setEnabled(True)

    def copy_code(self) -> None:
        if not self.generated_code:
            return
        QApplication.clipboard().setText(self.generated_code)
        message = "代码已复制到剪贴板。"
        self.warning_label.setText("\n".join([self._mode_message(), *self._warnings, message]))
