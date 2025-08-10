#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO模型训练脚本 - 竞赛障碍物检测
Training script for competition obstacle detection using YOLO
"""

import os
import sys
import yaml
import argparse
from ultralytics import YOLO
import torch
from pathlib import Path

class CompetitionYOLOTrainer:
    def __init__(self, config_path):
        """初始化训练器"""
        self.config = self.load_config(config_path)
        self.setup_environment()
        
    def load_config(self, config_path):
        """加载训练配置"""
        with open(config_path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    
    def setup_environment(self):
        """设置训练环境"""
        # 创建必要的目录
        os.makedirs(self.config['paths']['output_dir'], exist_ok=True)
        os.makedirs(self.config['paths']['model_backup_dir'], exist_ok=True)
        
        # 设置GPU
        if torch.cuda.is_available():
            print(f"GPU可用: {torch.cuda.get_device_name()}")
            self.device = 0
        else:
            print("使用CPU训练")
            self.device = 'cpu'
    
    def prepare_dataset(self):
        """准备数据集"""
        dataset_config = self.config['dataset']
        
        # 创建dataset.yaml文件
        dataset_yaml = {
            'path': dataset_config['path'],
            'train': dataset_config['train_split'],
            'val': dataset_config['val_split'],
            'test': dataset_config.get('test_split', 'test/images'),
            'nc': len(dataset_config['classes']),
            'names': dataset_config['classes']
        }
        
        dataset_yaml_path = os.path.join(dataset_config['path'], 'dataset.yaml')
        with open(dataset_yaml_path, 'w', encoding='utf-8') as f:
            yaml.dump(dataset_yaml, f, default_flow_style=False)
        
        print(f"数据集配置已保存到: {dataset_yaml_path}")
        return dataset_yaml_path
    
    def train_model(self, model_name='yolov8n'):
        """训练模型"""
        train_config = self.config['training']
        dataset_yaml = self.prepare_dataset()
        
        # 加载预训练模型
        model = YOLO(f'{model_name}.pt')
        
        # 开始训练
        print(f"开始训练模型: {model_name}")
        results = model.train(
            data=dataset_yaml,
            epochs=train_config['epochs'],
            batch=train_config['batch_size'],
            imgsz=train_config['image_size'],
            device=self.device,
            workers=train_config.get('workers', 8),
            project=self.config['paths']['output_dir'],
            name=train_config['experiment_name'],
            
            # 优化器设置
            optimizer=train_config.get('optimizer', 'AdamW'),
            lr0=train_config.get('learning_rate', 0.001),
            
            # 数据增强
            hsv_h=train_config.get('hsv_h', 0.015),
            hsv_s=train_config.get('hsv_s', 0.7),
            hsv_v=train_config.get('hsv_v', 0.4),
            degrees=train_config.get('degrees', 45.0),
            translate=train_config.get('translate', 0.2),
            scale=train_config.get('scale', 0.5),
            mosaic=train_config.get('mosaic', 1.0),
            mixup=train_config.get('mixup', 0.1),
            
            # 验证设置
            val=True,
            save=True,
            save_period=train_config.get('save_period', 50),
            
            # 早停设置
            patience=train_config.get('patience', 100),
        )
        
        return results
    
    def validate_model(self, model_path, dataset_yaml):
        """验证模型性能"""
        print(f"验证模型: {model_path}")
        
        model = YOLO(model_path)
        results = model.val(
            data=dataset_yaml,
            device=self.device,
            imgsz=self.config['training']['image_size']
        )
        
        # 提取关键指标
        metrics = {
            'mAP50-95': results.box.map,
            'mAP50': results.box.map50,
            'mAP75': results.box.map75,
            'precision': results.box.mp,
            'recall': results.box.mr
        }
        
        print("验证结果:")
        for metric, value in metrics.items():
            print(f"  {metric}: {value:.4f}")
        
        return metrics
    
    def export_model(self, model_path, export_formats=['onnx']):
        """导出模型为不同格式"""
        model = YOLO(model_path)
        
        for format_type in export_formats:
            print(f"导出为 {format_type} 格式...")
            try:
                model.export(
                    format=format_type,
                    device=self.device,
                    imgsz=self.config['training']['image_size']
                )
                print(f"  {format_type} 导出成功")
            except Exception as e:
                print(f"  {format_type} 导出失败: {e}")
    
    def benchmark_model(self, model_path):
        """性能基准测试"""
        model = YOLO(model_path)
        
        print("运行性能基准测试...")
        results = model.benchmark(
            device=self.device,
            imgsz=self.config['training']['image_size'],
            verbose=True
        )
        
        return results

def main():
    parser = argparse.ArgumentParser(description='竞赛YOLO模型训练')
    parser.add_argument('--config', type=str, required=True, help='训练配置文件路径')
    parser.add_argument('--model', type=str, default='yolov8n', 
                       choices=['yolov8n', 'yolov8s', 'yolov8m', 'yolov8l', 'yolov8x'],
                       help='YOLO模型类型')
    parser.add_argument('--export', action='store_true', help='训练后导出模型')
    parser.add_argument('--benchmark', action='store_true', help='运行性能基准测试')
    
    args = parser.parse_args()
    
    # 初始化训练器
    trainer = CompetitionYOLOTrainer(args.config)
    
    # 训练模型
    results = trainer.train_model(args.model)
    
    # 获取最佳模型路径
    best_model_path = results.save_dir / 'weights' / 'best.pt'
    
    # 验证模型
    dataset_yaml = trainer.prepare_dataset()
    metrics = trainer.validate_model(best_model_path, dataset_yaml)
    
    # 检查性能要求
    min_map50 = trainer.config.get('validation', {}).get('min_map50', 0.8)
    if metrics['mAP50'] >= min_map50:
        print(f"✅ 模型达到性能要求 (mAP50: {metrics['mAP50']:.4f} >= {min_map50})")
        
        # 导出模型
        if args.export:
            export_formats = trainer.config.get('export', {}).get('formats', ['onnx'])
            trainer.export_model(best_model_path, export_formats)
        
        # 性能基准测试
        if args.benchmark:
            trainer.benchmark_model(best_model_path)
            
    else:
        print(f"❌ 模型未达到性能要求 (mAP50: {metrics['mAP50']:.4f} < {min_map50})")
        print("建议:")
        print("1. 增加训练数据")
        print("2. 调整数据增强参数")
        print("3. 使用更大的模型")
        print("4. 增加训练轮数")

if __name__ == "__main__":
    main()