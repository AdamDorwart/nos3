#!/usr/bin/env python3

import os
import glob
import shutil
import argparse
from pathlib import Path

def process_text_files(directory):
    """Process all .txt files in the directory"""
    for txt_file in glob.glob(f"{directory}/**/*.txt", recursive=True):
        with open(txt_file, 'r') as f:
            content = f.read()
        
        content = content.replace('<%= CosmosCfsConfig::PROCESSOR_ENDIAN %>', 'LITTLE_ENDIAN')
        content = content.replace('<%=CF_INCOMING_PDU_MID%>', '0x1800')
        content = content.replace('<%=CF_SPACE_TO_GND_PDU_MID%>', '0x0800')
        
        with open(txt_file, 'w') as f:
            f.write(content)

def find_and_copy_targets(base_dir, gsw_dir, plugin_dir):
    """Find all target.txt files and copy their parent directories"""
    targets = set()
    
    # Find and copy from BASE_DIR
    for target_txt in glob.glob(f"{base_dir}/components/**/target.txt", recursive=True):
        target_dir = os.path.dirname(target_txt)
        target_name = os.path.basename(target_dir)
        targets.add(target_name)
        print(target_dir)
        shutil.copytree(target_dir, f"{plugin_dir}/targets/{target_name}", dirs_exist_ok=True)

    # Find and copy from GSW_DIR
    for target_txt in glob.glob(f"{gsw_dir}/config/targets/**/target.txt", recursive=True):
        target_dir = os.path.dirname(target_txt)
        target_name = os.path.basename(target_dir)
        targets.add(target_name)
        print(target_dir)
        shutil.copytree(target_dir, f"{plugin_dir}/targets/{target_name}", dirs_exist_ok=True)
    
    return sorted(list(targets))

def generate_plugin_txt(plugin_dir, targets):
    """Generate the plugin.txt file"""
    special_targets = {'SIM_42_TRUTH', 'SYSTEM', 'TO_DEBUG'}
    regular_targets = [t for t in targets if t not in special_targets]
    
    lines = []
    
    # Regular targets
    for target in regular_targets:
        lines.extend([
            f"TARGET {target} {target}_DEBUG",
            f"TARGET {target} {target}_RADIO"
        ])
    
    # Special targets
    for target in special_targets & set(targets):  # Intersection
        lines.append(f"TARGET {target} {target}")
    
    lines.extend([
        "",
        "INTERFACE DEBUG udp_interface.rb nos_fsw 5012 5013 nil nil 128 10.0 nil"
    ])
    
    # DEBUG mappings
    for target in regular_targets:
        lines.append(f"   MAP_TARGET {target}_DEBUG")
    lines.append("   MAP_TARGET TO_DEBUG")
    
    lines.extend([
        "",
        "INTERFACE RADIO udp_interface.rb radio_sim 6010 6011 nil nil 128 10.0 nil"
    ])
    
    # RADIO mappings
    for target in regular_targets:
        lines.append(f"   MAP_TARGET {target}_RADIO")
    
    lines.extend([
        "",
        "INTERFACE SIM_42_TRUTH_INT udp_interface.rb host.docker.internal 5110 5111 nil nil 128 10.0 nil",
        "   MAP_TARGET SIM_42_TRUTH"
    ])
    
    with open(f"{plugin_dir}/plugin.txt", 'w') as f:
        f.write('\n'.join(lines))

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--base-dir', required=True)
    parser.add_argument('--gsw-dir', required=True)
    parser.add_argument('--plugin-dir', required=True)
    args = parser.parse_args()
    
    targets = find_and_copy_targets(args.base_dir, args.gsw_dir, args.plugin_dir)
    process_text_files(f"{args.plugin_dir}/targets")
    generate_plugin_txt(args.plugin_dir, targets)

if __name__ == '__main__':
    main()
