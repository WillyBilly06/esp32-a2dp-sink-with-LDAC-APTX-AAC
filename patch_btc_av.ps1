$file = "D:\esp-idf\components\bt\host\bluedroid\btc\profile\std\a2dp\btc_av.c"
$content = Get-Content $file -Raw

$oldCode = @"
    case BTC_AV_SINK_CONFIG_REQ_EVT: {
        if (btc_av_cb.peer_sep == AVDT_TSEP_SRC) {
            esp_a2d_cb_param_t param;
            memcpy(param.audio_cfg.remote_bda, &btc_av_cb.peer_bda, sizeof(esp_bd_addr_t));
            memcpy(&param.audio_cfg.mcc, p_data, sizeof(esp_a2d_mcc_t));
            btc_a2d_cb_to_app(ESP_A2D_AUDIO_CFG_EVT, &param);
        }
    } break;

    default:
"@

$newCode = @"
    case BTC_AV_SINK_CONFIG_REQ_EVT: {
        if (btc_av_cb.peer_sep == AVDT_TSEP_SRC) {
            esp_a2d_cb_param_t param;
            memcpy(param.audio_cfg.remote_bda, &btc_av_cb.peer_bda, sizeof(esp_bd_addr_t));
            memcpy(&param.audio_cfg.mcc, p_data, sizeof(esp_a2d_mcc_t));
            btc_a2d_cb_to_app(ESP_A2D_AUDIO_CFG_EVT, &param);
        }
    } break;

    case BTA_AV_REJECT_EVT:
        /* Handle reject in idle state - this can happen during LDAC/aptX codec negotiation
         * when the first stream attempt fails. Log it but don't disconnect - allow retry. */
        BTC_TRACE_WARNING("%s : BTA_AV_REJECT_EVT in idle state - codec negotiation may retry\n", __FUNCTION__);
        break;

    default:
"@

if ($content.Contains("case BTC_AV_SINK_CONFIG_REQ_EVT:")) {
    $content = $content.Replace($oldCode, $newCode)
    Set-Content $file -Value $content -NoNewline
    Write-Host "SUCCESS: Patched btc_av.c - Added BTA_AV_REJECT_EVT handling in idle state"
} else {
    Write-Host "ERROR: Could not find target code block"
}
