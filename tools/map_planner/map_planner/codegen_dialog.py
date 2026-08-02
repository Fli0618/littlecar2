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
        self.copy_button = QPushButton("复制代码")
        self.close_button = QPushButton("关闭")
        self.regenerate_button.clicked.connect(self.regenerate)
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
        buttons.addStretch()
        buttons.addWidget(self.regenerate_button)
        buttons.addWidget(self.copy_button)
        buttons.addWidget(self.close_button)
        layout.addLayout(buttons)
        self.regenerate()

    def regenerate(self) -> None:
        """Regenerate preview while preserving the last valid code on errors."""

        try:
            self._warnings = validate_plan_for_blocking_codegen(self.plan)
            code = generate_task_function(self.plan, self.function_name_edit.text())
        except CodeGenerationError as error:
            self.warning_label.setText(str(error))
            self.copy_button.setEnabled(False)
            return

        self.generated_code = code
        self.code_preview.setPlainText(code)
        self.warning_label.setText("\n".join(self._warnings))
        self.copy_button.setEnabled(True)

    def copy_code(self) -> None:
        if not self.generated_code:
            return
        QApplication.clipboard().setText(self.generated_code)
        message = "代码已复制到剪贴板。"
        self.warning_label.setText("\n".join([*self._warnings, message]))
