.PHONY: setup teleop teleop-dual validate test render clean help

CONDA_ENV ?= pico_teleop
ACTIVATE  := source $(CONDA_PREFIX)/etc/profile.d/conda.sh && conda activate $(CONDA_ENV)

help:
	@echo "可用目标："
	@echo "  make setup        一键搭建环境（conda + 模型 + SDK + 依赖）"
	@echo "  make teleop       单臂遥操作 PICO -> PiPER（需先启动 PC 服务并连接头显）"
	@echo "  make teleop-dual  双臂遥操作（桌面双 PiPER + 抓取物）"
	@echo "  make validate     无头流水线验证（mock SDK）"
	@echo "  make test         运行 pytest 测试"
	@echo "  make render       离屏渲染场景为 PNG（docs/）"
	@echo "  make clean        清理构建产物"

setup:
	bash scripts/setup_env.sh

teleop:
	$(ACTIVATE) && python -m piper_pico

teleop-dual:
	$(ACTIVATE) && python -m piper_pico --dual

validate:
	$(ACTIVATE) && python tests/validate_piper_pipeline.py

test:
	$(ACTIVATE) && env -u PYTHONPATH pytest

render:
	$(ACTIVATE) && MUJOCO_GL=egl python scripts/render_scene.py

clean:
	rm -rf build dist *.egg-info piper_pico.egg-info
	find . -type d -name __pycache__ -exec rm -rf {} +
	find . -type d -name .pytest_cache -exec rm -rf {} +
